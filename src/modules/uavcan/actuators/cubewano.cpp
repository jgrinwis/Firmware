/****************************************************************************
 *
 *   Copyright (C) 2014 PX4 Development Team. All rights reserved.
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
 * @file cubewano.cpp
 *
 * @author Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include "cubewano.hpp"
#include <systemlib/err.h>

UavcanCubewanoController::UavcanCubewanoController(uavcan::INode &node) :
	_node(node),
	_uavcan_pub_cubewano_cmd(node),
	_uavcan_sub_status1(node),
	_uavcan_sub_status2(node),
	_uavcan_sub_status3(node),
	_orb_timer(node)
{
}

UavcanCubewanoController::~UavcanCubewanoController()
{
	perf_free(_perfcnt_invalid_input);
	perf_free(_perfcnt_scaling_error);
}

int UavcanCubewanoController::init()
{
	// cubewano status subscription
	int res1 = _uavcan_sub_status1.start(Status1CbBinder(this, &UavcanCubewanoController::cubewano_status1_sub_cb));
	if (res1 < 0)
	{
		warnx("cubewano status 1 sub failed %i", res1);
		return res1;
	}

	int res2 = _uavcan_sub_status2.start(Status2CbBinder(this, &UavcanCubewanoController::cubewano_status2_sub_cb));
	if (res2 < 0)
	{
		warnx("cubewano status 2 sub failed %i", res2);
		return res2;
	}

	int res3 = _uavcan_sub_status3.start(Status3CbBinder(this, &UavcanCubewanoController::cubewano_status3_sub_cb));
	if (res3 < 0)
	{
		warnx("cubewano status 3 sub failed %i", res3);
		return res3;
	}

	// cubewano status will be relayed from UAVCAN bus into ORB at this rate
	_orb_timer.setCallback(TimerCbBinder(this, &UavcanCubewanoController::orb_pub_timer_cb));
	_orb_timer.startPeriodic(uavcan::MonotonicDuration::fromMSec(1000 / cubewano_STATUS_UPDATE_RATE_HZ));

	return res3;
}

void UavcanCubewanoController::update_outputs(cubewano_commands_s& cmd)
{
	/*
	 * Rate limiting - we don't want to congest the bus
	 */
	const auto timestamp = _node.getMonotonicTime();
	if ((timestamp - _prev_cmd_pub).toUSec() < (1000000 / MAX_RATE_HZ)) {
		return;
	}
	_prev_cmd_pub = timestamp;

	/*
	 * Fill the command message
	 * If unarmed, we publish an empty message anyway
	 */
	uavcan::equipment::cubewano::CubewanoECUCommand msg;

//	static const int cmd_max = uavcan::equipment::cubewano::RawCommand::FieldTypes::cmd::RawValueType::max();

	msg.engine_ignition = cmd.ingnition;  //  set by mavlink cmd

//	if (_armed) {
//		for (unsigned i = 0; i < num_outputs; i++) {
//
//			float scaled = (outputs[i] + 1.0F) * 0.5F * cmd_max;
//			if (scaled < 1.0F) {
//				scaled = 1.0F;  // Since we're armed, we don't want to stop it completely
//			}
//
//			if (scaled > cmd_max) {
//				scaled = cmd_max;
//				perf_count(_perfcnt_scaling_error);
//			}
//
//			msg.engine_ignition = mavlink_ignition_val;  //  set by mavlink cmd
//
//			_cubewano_status.cubewano[i].cubewano_setpoint_raw = abs(static_cast<int>(scaled));
//		}
//	}

	/*
	 * Publish the command message to the bus
	 * Note that for a quadrotor it takes one CAN frame
	 */
	(void)_uavcan_pub_cubewano_cmd.broadcast(msg);
}


void UavcanCubewanoController::cubewano_status1_sub_cb(const uavcan::ReceivedDataStructure<uavcan::equipment::cubewano::CubewanoStatus1> &msg)
{
//	if (msg.cubewano_index < CONNECTED_cubewano_MAX) {
//		_cubewano_status.cubewano_count = uavcan::max<int>(_cubewano_status.cubewano_count, msg.cubewano_index + 1);
//		_cubewano_status.timestamp = msg.getMonotonicTimestamp().toUSec();
//
//		auto &ref = _cubewano_status.cubewano[msg.cubewano_index];
//
//		ref.cubewano_address = msg.getSrcNodeID().get();
//
//		ref.cubewano_voltage     = msg.voltage;
//		ref.cubewano_current     = msg.current;
//		ref.cubewano_temperature = msg.temperature;
//		ref.cubewano_setpoint    = msg.power_rating_pct;
//		ref.cubewano_rpm         = msg.rpm;
//		ref.cubewano_errorcount  = msg.error_count;
//	}
}


void UavcanCubewanoController::cubewano_status2_sub_cb(const uavcan::ReceivedDataStructure<uavcan::equipment::cubewano::CubewanoStatus2> &msg)
{
//	if (msg.cubewano_index < CONNECTED_cubewano_MAX) {
//		_cubewano_status.cubewano_count = uavcan::max<int>(_cubewano_status.cubewano_count, msg.cubewano_index + 1);
//		_cubewano_status.timestamp = msg.getMonotonicTimestamp().toUSec();
//
//		auto &ref = _cubewano_status.cubewano[msg.cubewano_index];
//
//		ref.cubewano_address = msg.getSrcNodeID().get();
//
//		ref.cubewano_voltage     = msg.voltage;
//		ref.cubewano_current     = msg.current;
//		ref.cubewano_temperature = msg.temperature;
//		ref.cubewano_setpoint    = msg.power_rating_pct;
//		ref.cubewano_rpm         = msg.rpm;
//		ref.cubewano_errorcount  = msg.error_count;
//	}
}


void UavcanCubewanoController::cubewano_status3_sub_cb(const uavcan::ReceivedDataStructure<uavcan::equipment::cubewano::CubewanoStatus3> &msg)
{
//	if (msg.cubewano_index < CONNECTED_cubewano_MAX) {
//		_cubewano_status.cubewano_count = uavcan::max<int>(_cubewano_status.cubewano_count, msg.cubewano_index + 1);
//		_cubewano_status.timestamp = msg.getMonotonicTimestamp().toUSec();
//
//		auto &ref = _cubewano_status.cubewano[msg.cubewano_index];
//
//		ref.cubewano_address = msg.getSrcNodeID().get();
//
//		ref.cubewano_voltage     = msg.voltage;
//		ref.cubewano_current     = msg.current;
//		ref.cubewano_temperature = msg.temperature;
//		ref.cubewano_setpoint    = msg.power_rating_pct;
//		ref.cubewano_rpm         = msg.rpm;
//		ref.cubewano_errorcount  = msg.error_count;
//	}
}

void UavcanCubewanoController::orb_pub_timer_cb(const uavcan::TimerEvent&)
{
	_cubewano_status.counter += 1;
	_cubewano_status.cubewano_connectiontype = CUBEWANO_CONNECTION_TYPE_CAN;

	if (_cubewano_status_pub > 0) {
		(void)orb_publish(ORB_ID(cubewano_status), _cubewano_status_pub, &_cubewano_status);
	} else {
		_cubewano_status_pub = orb_advertise(ORB_ID(cubewano_status), &_cubewano_status);
	}
}
