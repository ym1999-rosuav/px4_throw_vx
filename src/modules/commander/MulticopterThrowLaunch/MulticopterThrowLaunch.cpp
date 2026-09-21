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
 * “被抛出”的初速度后，等待飞机下落（垂速由负变正、即开始下坠）的瞬间自动
 * 启动电机，从而完成起飞。这样可以在没有平整起飞场地时快速起飞。
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
 *                                           │ (z 轴速度 > 0，检测到开始下坠)
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
	// 参数 COM_THROW_EN 是总开关：只有打开时才进入抛飞逻辑。
	if (_param_com_throw_en.get()) {
		// 每次调用都尝试获取最新的本地位置/速度；只有订阅数据有更新时才拷贝。
		// vx/vy/vz 为 NED 坐标系下的速度（z 向下为正，因此下坠时 vz > 0）。
		if (_vehicle_local_position_sub.updated()) {
			vehicle_local_position_s vehicle_local_position{};

			if (_vehicle_local_position_sub.copy(&vehicle_local_position)) {
				// 缓存当前速度向量，供 ARMED/UNSAFE 状态判断使用。
				_last_velocity = matrix::Vector3f(vehicle_local_position.vx, vehicle_local_position.vy, vehicle_local_position.vz);
			}
		}

		// 安全保护：抛飞过程中一旦上锁，立即中止并复位到 IDLE。
		// 此时处于“已抛出但电机可能尚未启动/正在启动”的危险窗口，必须提示用户。
		if (!armed && _throw_launch_state != ThrowLaunchState::IDLE) {
			events::send(events::ID("mc_throw_launch_not_ready"), events::Log::Critical, "Disarmed, don't throw");
			_throw_launch_state = ThrowLaunchState::IDLE;
		}

		// 状态机主体：每个 case 只处理“迁移条件”，不满足则保持原状态。
		switch (_throw_launch_state) {
		case ThrowLaunchState::IDLE:
			// IDLE -> ARMED：只要解锁，就进入“已就绪、等待被抛出”状态。
			if (armed) {
				events::send(events::ID("mc_throw_launch_ready"), events::Log::Critical, "Ready for throw launch");
				_throw_launch_state = ThrowLaunchState::ARMED;
			}

			break;

		case ThrowLaunchState::ARMED:
			// ARMED -> UNSAFE：速度模长超过阈值 COM_THROW_SPEED，
			// 认为操作者已经用力把飞机抛出（检测到抛投动作）。
			if (_last_velocity.longerThan(_param_com_throw_min_speed.get())) {
				PX4_INFO("Throw detected, motors will start once falling");
				_throw_launch_state = ThrowLaunchState::UNSAFE;
			}

			break;

		case ThrowLaunchState::UNSAFE:
			// UNSAFE -> FLYING：等待飞机达到抛物线的最高点并开始下坠。
			// NED 下 z 轴向下为正，故 vz > 0 表示“开始下落”，
			// 此刻才是启动电机的最佳时机（此时旋翼面向下、气流稳定）。
			if (_last_velocity(2) > 0.f) {
				PX4_INFO("Throw and fall detected, starting motors");
				_throw_launch_state = ThrowLaunchState::FLYING;
			}

			break;

		case ThrowLaunchState::DISABLED:
		case ThrowLaunchState::FLYING:
			// 功能关闭或已正常起飞，无需任何处理。
			break;
		}

	} else if (_throw_launch_state != ThrowLaunchState::DISABLED) {
		// 功能被关闭（COM_THROW_EN=0）时，确保状态复位到 DISABLED，
		// 避免残留的 ARMED/UNSAFE 状态影响下一次使能。
		_throw_launch_state = ThrowLaunchState::DISABLED;
	}
}
