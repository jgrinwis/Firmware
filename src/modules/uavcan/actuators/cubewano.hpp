/****************************************************************************
 *
 *   Copyright (C) 2014 Syzygyx Inc.
 *
 *   Reuse by third parties is not allowed.
 *
 ****************************************************************************/

/**
 * @file cubewano.hpp
 *
 * UAVCAN <--> Cubewano Engine messages:
 *     uavcan.equipment.cubewano.RawCommand
 *     uavcan.equipment.cubewano.RPMCommand
 *     uavcan.equipment.cubewano.Status
 *
 * @author Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#include <uavcan/uavcan.hpp>
#include <uavcan/equipment/cubewano/CubewanoECUCommand.hpp>
#include <uavcan/equipment/cubewano/CubewanoStatus1.hpp>
#include <uavcan/equipment/cubewano/CubewanoStatus2.hpp>
#include <uavcan/equipment/cubewano/CubewanoStatus3.hpp>
#include <systemlib/perf_counter.h>
#include <uORB/topics/cubewano_status.h>


class UavcanCubewanoController
{
public:
	UavcanCubewanoController(uavcan::INode& node);
	~UavcanCubewanoController();

	int init();

	void update_outputs(cubewano_commands_s&);

	void arm_cubewano(bool arm);
	void disarm_cubewano(bool disarm);

private:
	/**
	 * cubewano status message reception will be reported via this callback.
	 */
	void cubewano_status1_sub_cb(const uavcan::ReceivedDataStructure<uavcan::equipment::cubewano::CubewanoStatus1> &msg);
	void cubewano_status2_sub_cb(const uavcan::ReceivedDataStructure<uavcan::equipment::cubewano::CubewanoStatus2> &msg);
	void cubewano_status3_sub_cb(const uavcan::ReceivedDataStructure<uavcan::equipment::cubewano::CubewanoStatus3> &msg);

	/**
	 * cubewano status will be published to ORB from this callback (fixed rate).
	 */
	void orb_pub_timer_cb(const uavcan::TimerEvent &event);


	static constexpr unsigned MAX_RATE_HZ = 200;			///< XXX make this configurable
	static constexpr unsigned cubewano_STATUS_UPDATE_RATE_HZ = 10;

	typedef uavcan::MethodBinder<UavcanCubewanoController*,
		void (UavcanCubewanoController::*)(const uavcan::ReceivedDataStructure<uavcan::equipment::cubewano::CubewanoStatus1>&)>
		Status1CbBinder;

	typedef uavcan::MethodBinder<UavcanCubewanoController*,
		void (UavcanCubewanoController::*)(const uavcan::ReceivedDataStructure<uavcan::equipment::cubewano::CubewanoStatus2>&)>
		Status2CbBinder;

	typedef uavcan::MethodBinder<UavcanCubewanoController*,
		void (UavcanCubewanoController::*)(const uavcan::ReceivedDataStructure<uavcan::equipment::cubewano::CubewanoStatus3>&)>
		Status3CbBinder;

	typedef uavcan::MethodBinder<UavcanCubewanoController*, void (UavcanCubewanoController::*)(const uavcan::TimerEvent&)>
		TimerCbBinder;

	bool		_armed = false;
	//bool		_disarmed = true;
	//  telemetry status struct, for publishing via ORB...
	cubewano_status_s	_cubewano_status;
	orb_advert_t	_cubewano_status_pub = -1;

	/*
	 * libuavcan related things
	 */
	uavcan::MonotonicTime							_prev_cmd_pub;   ///< rate limiting
	uavcan::INode								&_node;
	uavcan::Publisher<uavcan::equipment::cubewano::CubewanoECUCommand>			_uavcan_pub_cubewano_cmd;
	uavcan::Subscriber<uavcan::equipment::cubewano::CubewanoStatus1, Status1CbBinder>	_uavcan_sub_status1;
	uavcan::Subscriber<uavcan::equipment::cubewano::CubewanoStatus2, Status2CbBinder>	_uavcan_sub_status2;
	uavcan::Subscriber<uavcan::equipment::cubewano::CubewanoStatus3, Status3CbBinder>	_uavcan_sub_status3;
	uavcan::TimerEventForwarder<TimerCbBinder>				_orb_timer;

	/*
	 * Perf counters
	 */
	perf_counter_t _perfcnt_invalid_input = perf_alloc(PC_COUNT, "uavcan_cubewano_invalid_input");
	perf_counter_t _perfcnt_scaling_error = perf_alloc(PC_COUNT, "uavcan_cubewano_scaling_error");
};
