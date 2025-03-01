/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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

#ifndef BATTERY_STATUS_HPP
#define BATTERY_STATUS_HPP

#include <uORB/topics/fuelcellbatt.h>

class MavlinkStreamBatteryStatus : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamBatteryStatus(mavlink); }

	static constexpr const char *get_name_static() { return "BATTERY_STATUS"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_BATTERY_STATUS; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		return MAVLINK_MSG_ID_BATTERY_STATUS_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
	}

private:
	explicit MavlinkStreamBatteryStatus(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _fuelcellbatt_sub{ORB_ID(fuelcellbatt)};

	bool send() override
	{
		mavlink_battery_status_t bat_msg{};
		bool updated = false;
		if(_fuelcellbatt_sub.advertised()){

			fuelcellbatt_s fuelcellbatt{};
			if(_fuelcellbatt_sub.copy(&fuelcellbatt)){

				bat_msg.voltages_ext[0]=fuelcellbatt.pila;
				bat_msg.voltages_ext[1]=fuelcellbatt.h2_pressure;
				bat_msg.voltages_ext[2]=fuelcellbatt.fan_speed;
				bat_msg.mode=fuelcellbatt.state;
				bat_msg.temperature=fuelcellbatt.top_temp;
				bat_msg.voltages[0]=fuelcellbatt.stack_voltage;
				bat_msg.voltages[1]=fuelcellbatt.battery_voltage;
				bat_msg.voltages[2]=fuelcellbatt.load_voltage;
				bat_msg.voltages[3]=fuelcellbatt.temp1;
				bat_msg.voltages[4]=fuelcellbatt.temp2;
				bat_msg.voltages[5]=fuelcellbatt.temp3;
				bat_msg.voltages[6]=fuelcellbatt.temp4;
				bat_msg.voltages[7]=fuelcellbatt.board_temp;
				bat_msg.voltages[8]=fuelcellbatt.target_temp;
				bat_msg.current_battery=fuelcellbatt.load_currente;
				bat_msg.time_remaining=fuelcellbatt.battery_current;
				bat_msg.current_consumed =fuelcellbatt.power;
                    bat_msg.energy_consumed = fuelcellbatt.energy;
				double mid=100*(-503.622+584.100*fuelcellbatt.battery_voltage/80.0-         252.554*pow(fuelcellbatt.battery_voltage/80.0,2)+48.204*pow(fuelcellbatt.battery_voltage/80.0,3)-3.422*pow(fuelcellbatt.battery_voltage/80.0,4));

				if (mid<100){
					if(mid<0){
						bat_msg.battery_remaining=0;
					}
					bat_msg.battery_remaining=mid;
				}
				else{
					bat_msg.battery_remaining=100;
				}
				bat_msg.type = fuelcellbatt.signo;
                		updated = true;
			}
		}
		if (updated) {
			mavlink_msg_battery_status_send_struct(_mavlink->get_channel(), &bat_msg);
			return true;
		}
		return false;
	}
};
#endif // BATTERY_STATUS_HPP
