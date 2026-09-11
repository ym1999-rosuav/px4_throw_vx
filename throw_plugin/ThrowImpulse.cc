#include <gz/plugin/Register.hh>

#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/components/ExternalWorldWrenchCmd.hh>

#include <gz/math/Vector3.hh>
#include <gz/math/Inertial.hh>

#include <gz/msgs/wrench.pb.h>
#include <gz/transport/Node.hh>
#include <gz/msgs/empty.pb.h>

#include <algorithm>
#include <chrono>
#include <iostream>
#include <fstream>
#include <mutex>
#include <string>

/// Simulates the "throw" of the vehicle: on a Gazebo transport message it applies
/// a short world wrench (force + torque) to the model canonical link, so the
/// simulated IMU senses the acceleration and PX4's estimator stays consistent.
/// PX4 then detects the throw through its throw-launch feature (COM_THROW_EN).
class ThrowImpulse :
    public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate
{
public:
    void Configure(
        const gz::sim::Entity &_entity,
        const std::shared_ptr<const sdf::Element> &_sdf,
        gz::sim::EntityComponentManager &/*_ecm*/,
        gz::sim::EventManager &) override
    {
        this->entity = _entity;

        if (_sdf->HasElement("model_name"))
            this->modelName = _sdf->Get<std::string>("model_name");

        if (_sdf->HasElement("topic"))
            this->topic = _sdf->Get<std::string>("topic");

        if (_sdf->HasElement("velocity"))
            this->throwVelocity = _sdf->Get<double>("velocity");

        if (_sdf->HasElement("acceleration"))
            this->throwAcceleration = _sdf->Get<double>("acceleration");

        if (_sdf->HasElement("roll_rate"))
            this->rollRate = _sdf->Get<double>("roll_rate");

        if (_sdf->HasElement("pitch_rate"))
            this->pitchRate = _sdf->Get<double>("pitch_rate");

        if (_sdf->HasElement("yaw_rate"))
            this->yawRate = _sdf->Get<double>("yaw_rate");

        if (this->node.Subscribe(this->topic, &ThrowImpulse::OnThrow, this)) {
            std::cout << "[ThrowImpulse] Listening on '" << this->topic
                      << "' (model=" << (this->modelName.empty() ? std::string("<self>") : this->modelName)
                      << ", velocity=" << this->throwVelocity << " m/s)" << std::endl;
            debugLog() << "[configure] topic=" << this->topic << " velocity=" << this->throwVelocity
                       << " accel=" << this->throwAcceleration
                       << " rates=" << this->rollRate << "," << this->pitchRate << "," << this->yawRate
                       << " model=" << this->modelName << std::endl;
        } else {
            std::cerr << "[ThrowImpulse] Failed to subscribe to '" << this->topic << "'" << std::endl;
        }
    }

    void OnThrow(const gz::msgs::Empty &/*_msg*/)
    {
        std::lock_guard<std::mutex> lock(this->mutex);
        this->pending = true;
        std::cout << "[ThrowImpulse] Throw trigger received" << std::endl;
        debugLog() << "[trigger] received" << std::endl;
    }

    void PreUpdate(
        const gz::sim::UpdateInfo &_info,
        gz::sim::EntityComponentManager &_ecm) override
    {
        if (_info.paused || this->done) {
            return;
        }

        const double now = std::chrono::duration<double>(_info.simTime).count();

        if (!this->launched) {
            std::lock_guard<std::mutex> lock(this->mutex);

            if (!this->pending) {
                return;
            }

            this->pending = false;

            if (!this->resolveLink(_ecm)) {
                return;
            }

            this->launched = true;
            this->launchStart = now;

            gz::sim::Link link(this->linkEntity);

            if (auto inertial = link.WorldInertial(_ecm)) {
                this->mass = std::max(inertial->MassMatrix().Mass(), 0.1);
                this->ixx = std::max(inertial->MassMatrix().Ixx(), 1e-4);
                this->iyy = std::max(inertial->MassMatrix().Iyy(), 1e-4);
                this->izz = std::max(inertial->MassMatrix().Izz(), 1e-4);
            }

            const double netAcceleration = std::max(this->throwAcceleration - 9.81, 1.0);
            this->impulseTime = this->throwVelocity / netAcceleration;

            debugLog() << "[throw] start: mass=" << this->mass << " impulse_time=" << this->impulseTime
                       << "s rates=" << this->rollRate << "," << this->pitchRate << "," << this->yawRate << std::endl;
        }

        gz::msgs::Wrench wrench;

        if ((now - this->launchStart) < this->impulseTime) {
            // Constant upward force + torque during the impulse window. Overwrite the
            // wrench command every step (do not accumulate).
            wrench.mutable_force()->set_z(this->mass * this->throwAcceleration);
            wrench.mutable_torque()->set_x(this->ixx * this->rollRate / this->impulseTime);
            wrench.mutable_torque()->set_y(this->iyy * this->pitchRate / this->impulseTime);
            wrench.mutable_torque()->set_z(this->izz * this->yawRate / this->impulseTime);
            this->applyWrench(_ecm, wrench);

        } else {
            this->applyWrench(_ecm, gz::msgs::Wrench());
            this->done = true;
            std::cout << "[ThrowImpulse] THROW! target Vz=" << this->throwVelocity << " m/s" << std::endl;
            debugLog() << "[throw] impulse done" << std::endl;
        }
    }

private:
    static std::ofstream &debugLog()
    {
        static std::ofstream log("/tmp/throw_plugin.log", std::ios::app);
        return log;
    }

    void applyWrench(gz::sim::EntityComponentManager &_ecm, const gz::msgs::Wrench &_wrench)
    {
        if (!_ecm.Component<gz::sim::components::ExternalWorldWrenchCmd>(this->linkEntity)) {
            _ecm.CreateComponent(this->linkEntity, gz::sim::components::ExternalWorldWrenchCmd());
        }

        _ecm.SetComponentData<gz::sim::components::ExternalWorldWrenchCmd>(this->linkEntity, _wrench);
    }

    bool resolveLink(gz::sim::EntityComponentManager &_ecm)
    {
        gz::sim::Entity modelEntity = gz::sim::kNullEntity;

        if (this->modelName.empty()) {
            gz::sim::Model selfModel(this->entity);

            if (selfModel.Valid(_ecm)) {
                modelEntity = this->entity;
            }
        }

        if (modelEntity == gz::sim::kNullEntity && !this->modelName.empty()) {
            auto modelOpt = _ecm.EntityByName(this->modelName);

            if (modelOpt.has_value()) {
                modelEntity = modelOpt.value();
            }
        }

        if (modelEntity == gz::sim::kNullEntity) {
            std::cerr << "[ThrowImpulse] Model not found, throw ignored" << std::endl;
            return false;
        }

        gz::sim::Model model(modelEntity);

        if (!model.Valid(_ecm)) {
            std::cerr << "[ThrowImpulse] Model entity invalid, throw ignored" << std::endl;
            return false;
        }

        this->linkEntity = model.CanonicalLink(_ecm);

        if (this->linkEntity == gz::sim::kNullEntity) {
            std::cerr << "[ThrowImpulse] Canonical link not found, throw ignored" << std::endl;
            return false;
        }

        return true;
    }

    gz::sim::Entity entity{gz::sim::kNullEntity};
    gz::sim::Entity linkEntity{gz::sim::kNullEntity};
    gz::transport::Node node;
    std::mutex mutex;

    std::string modelName{};
    std::string topic{"/throw"};
    double throwVelocity{6.0};
    double throwAcceleration{60.0};
    double rollRate{0.0};
    double pitchRate{0.0};
    double yawRate{0.0};
    double launchStart{0.0};
    double impulseTime{0.12};
    double mass{1.5};
    double ixx{0.03};
    double iyy{0.03};
    double izz{0.05};
    bool pending{false};
    bool launched{false};
    bool done{false};
};

GZ_ADD_PLUGIN(
    ThrowImpulse,
    gz::sim::System,
    ThrowImpulse::ISystemConfigure,
    ThrowImpulse::ISystemPreUpdate)
