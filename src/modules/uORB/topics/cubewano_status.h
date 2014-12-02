/****************************************************************************
 *
 *   Copyright (C) 2013 PX4 Development Team. All rights reserved.
 *   Author: @author Marco Bauer <marco@wtns.de>
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
 * @file cubewano_status.h
 * Definition of the cubewano_status uORB topic.
 *
 * Published the state machine and the system status bitfields
 * (see SYS_STATUS mavlink message), published only by commander app.
 *
 * All apps should write to subsystem_info:
 *
 *  (any app) --> subsystem_info (published) --> (commander app state machine)  --> cubewano_status --> (mavlink app)
 */

#ifndef CUBEWANO_STATUS_H_
#define CUBEWANO_STATUS_H_

#include <stdint.h>
#include <stdbool.h>
#include "../uORB.h"

/**
 * The number of CUBEWANO connections supported.
 * Currently: 1
 */
#define CONNECTED_CUBEWANO_MAX   1

enum CUBEWANO_VENDOR {
	CUBEWANO_VENDOR_GENERIC = 0,					/**< generic CUBEWANO */
};

enum CUBEWANO_CONNECTION_TYPE {
	CUBEWANO_CONNECTION_TYPE_PPM = 0,			/**< Traditional PPM CUBEWANO */
	CUBEWANO_CONNECTION_TYPE_SERIAL,			/**< Serial Bus connected CUBEWANO */
	CUBEWANO_CONNECTION_TYPE_ONESHOOT,			/**< One Shoot PPM */
	CUBEWANO_CONNECTION_TYPE_I2C,				/**< I2C */
	CUBEWANO_CONNECTION_TYPE_CAN				/**< CAN-Bus */
};

/**
 * @addtogroup topics
 * @{
 */

/**
 * Cubewano Engine status
 * Unsupported float fields should be assigned NaN.
 */
struct cubewano_status_s {
	/* use of a counter and timestamp recommended (but not necessary) */

	uint16_t counter;   		/**< incremented by the writing thread everytime new data is stored */
	uint64_t timestamp; 		/**< in microseconds since system start, is set whenever the writing thread stores new data */

	uint8_t cubewano_count;			/**< number of connected CUBEWANOs */
	enum CUBEWANO_CONNECTION_TYPE cubewano_connectiontype;	/**< how CUBEWANOs connected to the system */

	struct {
		enum CUBEWANO_VENDOR cubewano_vendor;		//  Vendor of current CUBEWANO
		uint32_t cubewano_errorcount;				//  Number of reported errors by CUBEWANO - if supported
        uint16_t cubewano_rpm;                      //  engine RPM
        uint16_t cubewano_MAP;						//  manifold air pressure, mBar
        uint16_t cubewano_batt_volt;			    //  battery voltage
        bool     cubewano_engine_state;				//  engine on/off state, 0 = on, 1 = off
        int16_t  cubewano_inlet_air_temp;			//  inlet air temp, deg C
        int16_t  cubewano_engine_temp;				//  engine temp (exhaust?), deg C
        uint16_t cubewano_fuel_type;				//  fuel type calibration, 1 = heavy, 2 = gas, 3 = other
        uint16_t cubewano_start_heater_state;		//  start heater on/off, 0 = off, 1 = on
        uint16_t cubewano_fuel_consumed;			//  fuel consumed (since restart?), liters
        uint16_t cubewano_fuel_final;  				//  ???,  unused
        int16_t	 cubewano_fuel_heat_temp;			//  fuel heater temp, if present, deg C
	} cubewano[CONNECTED_CUBEWANO_MAX];

};


//  place this in orb file to make struct def available to mavlink cmd handler
struct cubewano_commands_s
{
	int16_t 	ingnition;		//  engine on/off
	uint16_t	rpm;			//  engine rpm
	uint16_t	rpm_percentage; //  engine rpm %
	uint16_t    fuel_type;		//  fuel type (heavy, gas, other)
};

/**
 * @}
 */

/* register this as object request broker structure */
//ORB_DECLARE(cubewano_status);
ORB_DECLARE_OPTIONAL(cubewano_status);

#endif
