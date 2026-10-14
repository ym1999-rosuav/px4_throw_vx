/****************************************************************************
 *
 *   Copyright (c) 2023 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file MulticopterThrowLaunch.cpp
 *
 * 多旋翼“抛飞”（Throw Launch）起飞功能的实现。
 *
 * 使用场景：操作者把飞行器通电、解锁后，用手将飞机抛向空中，飞控检测到
 * “被抛出”的初速度后，记录当前位置；当飞机继续上升到设定高度时提前启动
 * 电机。若高度条件不可用，则保留开始下坠作为兜底条件。
 *
 * 状态机总览：
 *
 *     DISABLED ──(参数 COM_THROW_EN=1)──▶ IDLE
 *         ▲                                 │ (armed)
 *         │(参数关闭 / 上锁)                 ▼
 *         └────────────────────────────── ARMED
 *                                           │ (速度 > COM_THROW_SPEED，检测到抛投)
 *                                           ▼
 *                                        UNSAFE
 *                                           │ (上升高度达到阈值，或检测到开始下坠)
 *                                           ▼
 *                                        FLYING
 *
 * 注意：本类只负责维护状态并发出事件/日志，真正的“电机是否允许启动”由
 * commander 的状态机根据 isThrowLaunchInProgress()/isReadyToThrow() 判断。
 *
 * @author Michał Barciś <mbarcis@mbarcis.net>
 */

#include "MulticopterThrowLaunch.hpp"
#include <px4_platform_common/events.h>

/**
 * 构造函数。
 *
 * @param parent 父级 ModuleParams，用于把本类中 DEFINE_PARAMETERS 定义的
 *               参数（COM_THROW_EN、COM_THROW_SPEED）挂到父模块的参数树上，
 *               从而支持运行时动态更新。
 */
MulticopterThrowLaunch::MulticopterThrowLaunch(ModuleParams *parent) :
	ModuleParams(parent)
{}

/**
 * 抛飞状态机的主更新函数，由 commander 在每次主循环中调用。
 *
 * @param armed 当前载具是否已解锁（true = 已解锁）
 *
 * 执行流程：
 *  1. 若功能未使能（COM_THROW_EN=0），强制回到 DISABLED 并返回。
 *  2. 若功能使能，则刷新最近一次本地位置速度（用于后续抛投检测）。
 *  3. 若在非 IDLE 状态下发生上锁，说明抛飞被中断，记录事件并复位到 IDLE。
 *  4. 根据当前状态推进状态机。
 */
void MulticopterThrowLaunch::update(const bool armed)
{
	bool local_position_updated = false;

	// 只在收到新的 EKF 本地位置样本时推进连续确认计时，避免把同一帧数据重复计数。
	if (_vehicle_local_position_sub.updated()) {
		vehicle_local_position_s vehicle_local_position{};

		if (_vehicle_local_position_sub.copy(&vehicle_local_position)) {
			_last_velocity = matrix::Vector3f(vehicle_local_position.vx, vehicle_local_position.vy,
							 vehicle_local_position.vz);
			_last_position_z = vehicle_local_position.z;
			_last_position_z_valid = vehicle_local_position.z_valid;
			_last_velocity_xy_valid = vehicle_local_position.v_xy_valid;
			_last_velocity_z_valid = vehicle_local_position.v_z_valid;
			_last_z_reset_counter = vehicle_local_position.z_reset_counter;
			local_position_updated = true;
		}
	}

	const hrt_abstime now = hrt_absolute_time();

	// 抛飞过程中不允许通过修改参数直接解除 lockdown，否则电机会在手中突然启动。
	if (!_param_com_throw_en.get()) {
		if (armed && isThrowLaunchInProgress()) {
			if (!_disable_while_armed_reported) {
				events::send(events::ID("mc_throw_disable_armed"), events::Log::Critical,
					     "Disarm before disabling throw launch");
				_disable_while_armed_reported = true;
			}

			return;
		}

		_throw_launch_state = ThrowLaunchState::DISABLED;
		_waiting_reference_z_valid = false;
		_throw_start_z_valid = false;
		_seen_real_climb = false;
		_height_condition_since = 0;
		_fall_condition_since = 0;
		_throw_detected_at = 0;
		_throw_timeout_reported = false;
		_disable_while_armed_reported = false;
		return;
	}

	_disable_while_armed_reported = false;

	// 参数在未解锁时启用，显式进入 IDLE。禁止在已经解锁后临时开启抛飞模式。
	if (_throw_launch_state == ThrowLaunchState::DISABLED) {
		if (armed) {
			if (!_enable_while_armed_reported) {
				events::send(events::ID("mc_throw_enable_armed"), events::Log::Critical,
					     "Disarm before enabling throw launch");
				_enable_while_armed_reported = true;
			}

			return;
		}

		_throw_launch_state = ThrowLaunchState::IDLE;
	}

	if (!armed) {
		_enable_while_armed_reported = false;

		if (isThrowLaunchInProgress()) {
			events::send(events::ID("mc_throw_launch_not_ready"), events::Log::Critical, "Disarmed, don't throw");
		}

		if (_throw_launch_state != ThrowLaunchState::IDLE) {
			_throw_launch_state = ThrowLaunchState::IDLE;
		}

		_waiting_reference_z_valid = false;
		_throw_start_z_valid = false;
		_seen_real_climb = false;
		_height_condition_since = 0;
		_fall_condition_since = 0;
		_throw_detected_at = 0;
		_throw_timeout_reported = false;
	}

	switch (_throw_launch_state) {
	case ThrowLaunchState::IDLE:
		if (armed) {
			events::send(events::ID("mc_throw_launch_ready"), events::Log::Critical, "Ready for throw launch");
			_waiting_reference_z_valid = false;
			_throw_start_z_valid = false;
			_seen_real_climb = false;
			_height_condition_since = 0;
			_fall_condition_since = 0;
			_throw_detected_at = 0;
			_throw_timeout_reported = false;
			_throw_launch_state = ThrowLaunchState::ARMED;
		}

		break;

	case ThrowLaunchState::ARMED:
		if (local_position_updated) {
			// Altitude 模式不要求水平位置/速度有效。没有 GPS 或光流时，
			// 只要气压高度和垂直速度有效，仍然可以进行向上手抛检测。
			if (!_last_position_z_valid || !_last_velocity_z_valid) {
				_waiting_reference_z_valid = false;
				break;
			}

			// 待抛期间用低通参考持续跟随慢速气压漂移；手臂开始加速后停止更新参考。
			if (!_waiting_reference_z_valid) {
				_waiting_reference_z = _last_position_z;
				_waiting_reference_z_valid = true;
			}

			const float throw_speed = _param_com_throw_min_speed.get();
			const bool throw_speed_reached = throw_speed > 0.f
							 && (_last_velocity_xy_valid
							     ? _last_velocity.longerThan(throw_speed)
							     : _last_velocity(2) <= -throw_speed);

			if (throw_speed_reached) {
				_throw_start_z = _waiting_reference_z;
				_throw_start_z_valid = _waiting_reference_z_valid;
				_throw_start_z_reset_counter = _last_z_reset_counter;
				_seen_real_climb = false;
				_height_condition_since = 0;
				_fall_condition_since = 0;
				_throw_detected_at = now;
				_throw_timeout_reported = false;
				PX4_INFO("Throw detected, waiting for confirmed height gain");
				_throw_launch_state = ThrowLaunchState::UNSAFE;

			} else {
				// 无水平速度辅助时，不能使用无效的 vx/vy 判断飞机是否静止；
				// 只用有效的垂直速度，使待抛高度参考继续跟随慢速气压漂移。
				const bool moving_slowly = _last_velocity_xy_valid
							   ? !_last_velocity.longerThan(0.5f)
							   : (_last_velocity(2) > -0.5f && _last_velocity(2) < 0.5f);

				if (moving_slowly) {
					// 约 5 个新样本的指数平滑；不会把解锁后的长期漂移积累为起飞高度。
					constexpr float reference_alpha = 0.2f;
					_waiting_reference_z += reference_alpha * (_last_position_z - _waiting_reference_z);
				}
			}
		}

		break;

	case ThrowLaunchState::UNSAFE: {
		const float confirm_time_s = _param_com_throw_confirm_time.get();
		const hrt_abstime confirm_time_us = confirm_time_s > 0.f
				? static_cast<hrt_abstime>(confirm_time_s * 1e6f) : 0;
		bool height_confirmed = false;
		bool fall_confirmed = false;

		if (local_position_updated) {
			const bool continuous_height = _throw_start_z_valid
						       && _last_position_z_valid
						       && _last_z_reset_counter == _throw_start_z_reset_counter;
			const float height_gain = continuous_height ? _throw_start_z - _last_position_z : 0.f;
			const float min_up_speed = _param_com_throw_min_up_speed.get();
			const float min_down_speed = _param_com_throw_min_down_speed.get();

			// 必须先看见至少 0.15 m 的真实上升，下降兜底才有资格生效。
			if (continuous_height && _last_velocity_z_valid && height_gain >= 0.15f
			    && _last_velocity(2) <= -min_up_speed) {
				_seen_real_climb = true;
			}

			const float throw_height = _param_com_throw_height.get();
			const bool height_candidate = throw_height > 0.f && continuous_height && _seen_real_climb
						      && _last_velocity_z_valid && _last_velocity(2) <= -min_up_speed
						      && height_gain >= throw_height;

			if (height_candidate) {
				if (_height_condition_since == 0) {
					_height_condition_since = now;
				}

				height_confirmed = confirm_time_us == 0 || now - _height_condition_since >= confirm_time_us;

			} else {
				_height_condition_since = 0;
			}

			const bool fall_candidate = _seen_real_climb && _last_velocity_z_valid
						    && _last_velocity(2) >= min_down_speed;

			if (fall_candidate) {
				if (_fall_condition_since == 0) {
					_fall_condition_since = now;
				}

				fall_confirmed = confirm_time_us == 0 || now - _fall_condition_since >= confirm_time_us;

			} else {
				_fall_condition_since = 0;
			}
		}

		if (height_confirmed || fall_confirmed) {
			if (height_confirmed) {
				PX4_INFO("Throw height confirmed, starting motors");

			} else {
				PX4_INFO("Throw descent confirmed, starting motors");
			}

			_throw_launch_state = ThrowLaunchState::FLYING;
			_height_condition_since = 0;
			_fall_condition_since = 0;
			break;
		}

		const float timeout_s = _param_com_throw_timeout.get();
		const hrt_abstime timeout_us = timeout_s > 0.f ? static_cast<hrt_abstime>(timeout_s * 1e6f) : 0;

		if (!_throw_timeout_reported && timeout_us > 0 && _throw_detected_at != 0
		    && now - _throw_detected_at >= timeout_us) {
			events::send(events::ID("mc_throw_launch_timeout"), events::Log::Critical,
				     "Throw not confirmed, disarm and retry");
			_throw_timeout_reported = true;
			// 故意保持 UNSAFE/lockdown：超时绝不能自动启动电机。
		}

		break;
	}

	case ThrowLaunchState::DISABLED:
	case ThrowLaunchState::FLYING:
		break;
	}
}
