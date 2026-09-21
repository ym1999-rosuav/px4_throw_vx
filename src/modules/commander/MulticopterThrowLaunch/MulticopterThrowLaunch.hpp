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
 * @file MulticopterThrowLaunch.hpp
 *
 * Changes to manage a takeoff of a multicopter by manually throwing it into the air.
 *
 * 多旋翼“抛飞”起飞功能的声明。
 *
 * 该类被 commander 模块持有，提供一个简单的状态机：操作者解锁后把飞机抛向
 * 空中，飞控检测到抛投速度并等待飞机开始下坠，随后允许电机启动完成起飞。
 * 具体状态迁移逻辑见 MulticopterThrowLaunch.cpp 的 update()。
 *
 * @author Michał Barciś <mbarcis@mbarcis.net>
 */

#pragma once

// matrix::Vector3f 等线性代数工具，用于速度向量运算（longerThan、取分量等）
#include <lib/matrix/matrix/math.hpp>
// ModuleParams：提供参数（DEFINE_PARAMETERS）机制
#include <px4_platform_common/module_params.h>

// uORB 订阅封装及本地位置话题，用于获取飞行器实时速度
#include <uORB/Subscription.hpp>
#include <uORB/topics/vehicle_local_position.h>

/**
 * 多旋翼抛飞状态机。
 *
 * 继承 ModuleParams 以便定义 COM_THROW_EN / COM_THROW_SPEED 两个参数。
 * 典型用法（在 commander 中）：
 *   - 每轮循环调用 update(armed) 推进状态；
 *   - 通过 isThrowLaunchInProgress() 判断是否处于抛飞流程中；
 *   - 通过 isReadyToThrow() 判断是否已就绪等待被抛出。
 */
class MulticopterThrowLaunch : public ModuleParams
{
public:
	/**
	 * 构造函数。
	 * @param parent 父级 ModuleParams，用于把本类的参数挂到父模块的参数树上。
	 */
	explicit MulticopterThrowLaunch(ModuleParams *parent);
	~MulticopterThrowLaunch() override = default;

	/**
	 * 是否正在进行抛飞流程。
	 * 用于 commander 判断是否需要屏蔽常规的“未解锁/地面”起飞逻辑。
	 *
	 * @return 功能未使能（DISABLED）或已完成起飞（FLYING）时返回 false，
	 *         其余状态（IDLE/ARMED/UNSAFE）返回 true。
	 */
	bool isThrowLaunchInProgress() const
	{
		return _throw_launch_state != ThrowLaunchState::DISABLED
		       && _throw_launch_state != ThrowLaunchState::FLYING;
	}

	/**
	 * 是否已就绪、等待操作者抛出。
	 * @return 仅当处于 ARMED 状态时返回 true。
	 */
	bool isReadyToThrow() const { return _throw_launch_state == ThrowLaunchState::ARMED; }

	/**
	 * 状态机主更新函数，由 commander 周期调用。
	 * @param armed 当前载具是否已解锁（true = 已解锁）
	 */
	void update(const bool armed);

	/**
	 * 抛飞流程的状态枚举。
	 *
	 * DISABLED：功能未使能（COM_THROW_EN=0），不参与抛飞逻辑。
	 * IDLE    ：功能已使能但未解锁，等待解锁。
	 * ARMED   ：已解锁，等待检测到抛投动作（速度超过阈值）。
	 * UNSAFE  ：检测到被抛出，等待飞机到达最高点并开始下坠。
	 * FLYING  ：检测到下坠，允许启动电机，抛飞完成。
	 */
	enum class ThrowLaunchState {
		DISABLED = 0,
		IDLE = 1,
		ARMED = 2,
		UNSAFE = 3,
		FLYING = 4
	};

private:
	// 订阅本地位置话题，用于获取 NED 坐标系下的实时速度（vx/vy/vz）
	uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};

	// 当前状态机状态，默认未使能
	ThrowLaunchState _throw_launch_state{ThrowLaunchState::DISABLED};
	// 最近一次从 vehicle_local_position 读取到的速度向量（NED，z 向下为正）
	matrix::Vector3f _last_velocity{};

	// 抛飞功能相关参数：
	//   COM_THROW_EN   —— 抛飞功能总开关（bool）
	//   COM_THROW_SPEED—— 判定“被抛出”的最小速度阈值（m/s）
	DEFINE_PARAMETERS(
		(ParamBool<px4::params::COM_THROW_EN>) _param_com_throw_en,
		(ParamFloat<px4::params::COM_THROW_SPEED>) _param_com_throw_min_speed
	);
};
