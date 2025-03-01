/****************************************************************************
 *
 *   Copyright (c) 2013-2023 PX4 Development Team. All rights reserved.
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

#include "FixedwingRateControl.hpp"

// #include <string>
#include <iostream>

#define MANUALPARAMSLQRLON 0

#define MANUALPARAMSLQRLAT 0


#define DEBUGCONTROLS 0


using namespace time_literals;
using namespace matrix;

using math::constrain;
using math::interpolate;
using math::radians;

FixedwingRateControl::FixedwingRateControl(bool vtol) :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers),
	_actuator_controls_status_pub(vtol ? ORB_ID(actuator_controls_status_1) : ORB_ID(actuator_controls_status_0)),
	_vehicle_torque_setpoint_pub(vtol ? ORB_ID(vehicle_torque_setpoint_virtual_fw) : ORB_ID(vehicle_torque_setpoint)),
	_vehicle_thrust_setpoint_pub(vtol ? ORB_ID(vehicle_thrust_setpoint_virtual_fw) : ORB_ID(vehicle_thrust_setpoint)),
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle"))
{
	_handle_param_vt_fw_difthr_en = param_find("VT_FW_DIFTHR_EN");

	/* fetch initial parameter values */
	parameters_update();

	_rate_ctrl_status_pub.advertise();
}

FixedwingRateControl::~FixedwingRateControl()
{
	perf_free(_loop_perf);
}

bool
FixedwingRateControl::init()
{
	if (!_vehicle_angular_velocity_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	return true;
}

int
FixedwingRateControl::parameters_update()
{
	const Vector3f rate_p = Vector3f(_param_fw_rr_p.get(), _param_fw_pr_p.get(), _param_fw_yr_p.get());
	const Vector3f rate_i = Vector3f(_param_fw_rr_i.get(), _param_fw_pr_i.get(), _param_fw_yr_i.get());
	const Vector3f rate_d = Vector3f(_param_fw_rr_d.get(), _param_fw_pr_d.get(), _param_fw_yr_d.get());

	_rate_control.setPidGains(rate_p, rate_i, rate_d);

	_rate_control.setIntegratorLimit(
		Vector3f(_param_fw_rr_imax.get(), _param_fw_pr_imax.get(), _param_fw_yr_imax.get()));

	_rate_control.setFeedForwardGain(
		// set FF gains to 0 as we add the FF control outside of the rate controller
		Vector3f(0.f, 0.f, 0.f));

	if (_handle_param_vt_fw_difthr_en != PARAM_INVALID) {
		param_get(_handle_param_vt_fw_difthr_en, &_param_vt_fw_difthr_en);
	}

	_lqr_lat_k11 = _param_fw_lqr_lat_k11.get();
	_lqr_lat_k12 = _param_fw_lqr_lat_k12.get();
	_lqr_lat_k13 = _param_fw_lqr_lat_k13.get();
	_lqr_lat_k14 = _param_fw_lqr_lat_k14.get();
	_lqr_lat_k21 = _param_fw_lqr_lat_k21.get();
	_lqr_lat_k22 = _param_fw_lqr_lat_k22.get();
	_lqr_lat_k23 = _param_fw_lqr_lat_k23.get();
	_lqr_lat_k24 = _param_fw_lqr_lat_k24.get();

	_lqr_lon_k1 = _param_fw_lqr_lon_k1.get();
	_lqr_lon_k2 = _param_fw_lqr_lon_k2.get();
	_lqr_lon_k3 = _param_fw_lqr_lon_k3.get();

	_deltaE_trim = _param_fw_deltaE_trim.get();
	_lqr_Ts = _param_fw_lqr_Ts.get();
	_rudder_max = _param_fw_rudder_max.get();
	_rudder_min = _param_fw_rudder_min.get();
	_theta_trim = _param_fw_theta_trim.get();

	return PX4_OK;
}

void
FixedwingRateControl::vehicle_manual_poll()
{
	if (_vcontrol_mode.flag_control_manual_enabled && _in_fw_or_transition_wo_tailsitter_transition) {

		// Always copy the new manual setpoint, even if it wasn't updated, to fill the actuators with valid values
		if (_manual_control_setpoint_sub.copy(&_manual_control_setpoint)) {

			if (_vcontrol_mode.flag_control_rates_enabled &&
			    !_vcontrol_mode.flag_control_attitude_enabled) {

				// RATE mode we need to generate the rate setpoint from manual user inputs

				if (_vehicle_status.is_vtol_tailsitter && _vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING) {
					// the rate_sp must always be published in body (hover) frame
					_rates_sp.roll = _manual_control_setpoint.yaw * radians(_param_fw_acro_z_max.get());
					_rates_sp.yaw = -_manual_control_setpoint.roll * radians(_param_fw_acro_x_max.get());

				} else {
					_rates_sp.roll = _manual_control_setpoint.roll * radians(_param_fw_acro_x_max.get());
					_rates_sp.yaw = _manual_control_setpoint.yaw * radians(_param_fw_acro_z_max.get());
				}

				_rates_sp.timestamp = hrt_absolute_time();
				_rates_sp.pitch = -_manual_control_setpoint.pitch * radians(_param_fw_acro_y_max.get());
				_rates_sp.thrust_body[0] = (_manual_control_setpoint.throttle + 1.f) * .5f;

				_rate_sp_pub.publish(_rates_sp);

			} else {
				// Manual/direct control, filled in FW-frame. Note that setpoints will get transformed to body frame prior publishing.

				_vehicle_torque_setpoint.xyz[0] = math::constrain(_manual_control_setpoint.roll * _param_fw_man_r_sc.get() +
								  _param_trim_roll.get(), -1.f, 1.f);
				_vehicle_torque_setpoint.xyz[1] = math::constrain(-_manual_control_setpoint.pitch * _param_fw_man_p_sc.get() +
								  _param_trim_pitch.get(), -1.f, 1.f);
				_vehicle_torque_setpoint.xyz[2] = math::constrain(_manual_control_setpoint.yaw * _param_fw_man_y_sc.get() +
								  _param_trim_yaw.get(), -1.f, 1.f);

				_vehicle_thrust_setpoint.xyz[0] = math::constrain((_manual_control_setpoint.throttle + 1.f) * .5f, 0.f, 1.f);
			}
		}
	}
}

void
FixedwingRateControl::vehicle_land_detected_poll()
{
	if (_vehicle_land_detected_sub.updated()) {
		vehicle_land_detected_s vehicle_land_detected {};

		if (_vehicle_land_detected_sub.copy(&vehicle_land_detected)) {
			_landed = vehicle_land_detected.landed;
		}
	}
}

float FixedwingRateControl::get_airspeed_and_update_scaling()
{
	_airspeed_validated_sub.update();
	const bool airspeed_valid = PX4_ISFINITE(_airspeed_validated_sub.get().calibrated_airspeed_m_s)
				    && (hrt_elapsed_time(&_airspeed_validated_sub.get().timestamp) < 1_s);

	// if no airspeed measurement is available out best guess is to use the trim airspeed
	float airspeed = _param_fw_airspd_trim.get();

	if (_param_fw_use_airspd.get() && airspeed_valid) {
		/* prevent numerical drama by requiring 0.5 m/s minimal speed */
		airspeed = math::max(0.5f, _airspeed_validated_sub.get().calibrated_airspeed_m_s);

	} else {
		// VTOL: if we have no airspeed available and we are in hover mode then assume the lowest airspeed possible
		// this assumption is good as long as the vehicle is not hovering in a headwind which is much larger
		// than the stall airspeed
		if (_vehicle_status.is_vtol && _vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING
		    && !_vehicle_status.in_transition_mode) {
			airspeed = _param_fw_airspd_stall.get();
		}
	}

	/*
	 * For scaling our actuators using anything less than the stall
	 * speed doesn't make any sense - its the strongest reasonable deflection we
	 * want to do in flight and it's the baseline a human pilot would choose.
	 *
	 * Forcing the scaling to this value allows reasonable handheld tests.
	 */
	const float airspeed_constrained = constrain(constrain(airspeed, _param_fw_airspd_stall.get(),
					   _param_fw_airspd_max.get()), 0.1f, 1000.0f);

	_airspeed_scaling = (_param_fw_arsp_scale_en.get()) ? (_param_fw_airspd_trim.get() / airspeed_constrained) : 1.0f;

	return airspeed;
}

void FixedwingRateControl::LQR(){

	bool params_updated = _parameter_update_sub.updated();

	// check for parameter updates
	if (params_updated) {
		// clear update
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);

		// update parameters from storage
		updateParams();
		parameters_update();
	}

	//Update attitude values and setppoints
	_att_sp_sub.update(&_att_sp);
	_att_sub.update(&_att);

	//Obtain euler angles
	const matrix::Eulerf euler_angles(matrix::Quatf(_att.q));

	//Rates setpoint is updated to obtain thrust setpoint
	_rates_sp_sub.update(&_rates_sp);

	/* throttle passed through if it is finite */
	_vehicle_thrust_setpoint.xyz[0] = PX4_ISFINITE(_rates_sp.thrust_body[0]) ? _rates_sp.thrust_body[0] : 0.0f;

	//Calculate roll error and apply antiwindup by stopping the update of roll error
	float roll_error = _att_sp.roll_body - euler_angles.phi();
	float ei_roll = (_antiwindup_flag) ? _ei_roll_1 : _ei_roll_1 + roll_error * _lqr_Ts;
	_ei_roll_1 = ei_roll;

	//Calculate pitch error
	float pitch_error = _att_sp.pitch_body - euler_angles.theta();
	float ei_pitch = _ei_pitch_1 + pitch_error*_lqr_Ts;
	_ei_pitch_1 = ei_pitch;

	//Obtain angular velocities and calculate body rates
	vehicle_angular_velocity_s angular_velocity{};
	Vector3f rates(angular_velocity.xyz);
	float p = rates(0) - rates(2)*sin(euler_angles.theta());
	float q = rates(1)*cos(euler_angles.phi()) + rates(2)*sin(euler_angles.phi())*cos(euler_angles.theta());
	float r = -rates(1)*sin(euler_angles.phi()) + rates(2)*cos(euler_angles.phi())*cos(euler_angles.theta());

//Pass manual parameters to facilitate calibration (easier to copy MATLAB values)
#if MANUALPARAMSLQRLON == 1

	float lonparametersvector[4] = {0.204294603869260,0.454370662399990,-0.223606797749979};


	_lqr_lon_k1 = lonparametersvector[0];
	_lqr_lon_k2 = lonparametersvector[1];
	_lqr_lon_k3 = lonparametersvector[2];
#endif

#if MANUALPARAMSLQRLAT == 1





	// float latparametersvector[8] = {2.876789712516354,0.099912971892887,5.495914566784522,-3.152166827888144,-0.116788291778550,2.666509893212911,0.526179710652797,-0.252674274831497};
	// float latparametersvector[8] = {1.151276595488395,0.092202999716521,2.496460880764733,-1.409703309694603,-0.044500071901226,0.978679626686552,0.280878869596752,-0.112856451459787};
// R = 1	// float latparametersvector[8] = {0.752457056934047,0.097712577957282,1.792317295083641,-0.997380740374279,-0.027675517311056,0.603827180992747,0.201123757781229,-0.072330206210520};

float latparametersvector[8] = {0.580104568523759,0.100418024217664,1.482408145739839,-0.814683629823900,-0.020434531300292,0.447446659911569,0.162646727992977,-0.054380602825102};


	_lqr_lat_k11 = latparametersvector[0];
	_lqr_lat_k12 = latparametersvector[1];
	_lqr_lat_k13 = latparametersvector[2];
	_lqr_lat_k14 = latparametersvector[3];
	_lqr_lat_k21 = latparametersvector[4];
	_lqr_lat_k22 = latparametersvector[5];
	_lqr_lat_k23 = latparametersvector[6];
	_lqr_lat_k24 = latparametersvector[7];
#endif

	//Aileron and rudder calculation (lateral LQR)
	//They are defined as torque variables, but they are angles
	_vehicle_torque_setpoint.xyz[0] = -1*(_lqr_lat_k11*p + _lqr_lat_k12*r + _lqr_lat_k13*euler_angles.phi() + _lqr_lat_k14*ei_roll);
	_vehicle_torque_setpoint.xyz[2] = -1*(_lqr_lat_k21*p + _lqr_lat_k22*r + _lqr_lat_k23*euler_angles.phi() + _lqr_lat_k24*ei_roll);

	//Antiwindup limits and flag
	if(_vehicle_torque_setpoint.xyz[2] > _rudder_max || _vehicle_torque_setpoint.xyz[2] < _rudder_min){
		_antiwindup_flag = 1;
		_vehicle_torque_setpoint.xyz[2] = math::constrain(_vehicle_torque_setpoint.xyz[2], _rudder_min, _rudder_max);
	}

	else
		_antiwindup_flag = 0;

	//Elevator calculation (longitudinal LQR)
	//Initial elevator deflection is added at the end as LQR calculates the increment from trim value
	//Initial pitch angle is subtracted to the current pitch angle to obtain the increment
	_vehicle_torque_setpoint.xyz[1] = -1*(_lqr_lon_k1*q + _lqr_lon_k2*(euler_angles.theta()-_theta_trim) + _lqr_lon_k3*ei_pitch) + _deltaE_trim;


	/* scale effort by battery status */
	if (_param_fw_bat_scale_en.get() && _vehicle_thrust_setpoint.xyz[0] > 0.1f) {

		if (_battery_status_sub.updated()) {
			battery_status_s battery_status{};

			if (_battery_status_sub.copy(&battery_status) && battery_status.connected && battery_status.scale > 0.f) {
				_battery_scale = battery_status.scale;
			}
		}

		_vehicle_thrust_setpoint.xyz[0] *= _battery_scale;
	}

//Print control values to consol for debug
#if DEBUGCONTROLS == 1
	static int ithrustlqr = 500;
	ithrustlqr++;
	if(ithrustlqr%1000==0)
		std::cout << "THRUST SETPOINT IS:" << _vehicle_thrust_setpoint.xyz[0] << std::endl;

	static int itorquelqr = 0;
	itorquelqr++;
	if(itorquelqr%1000==0)
		std::cout << "TORQUE SETPOINT IS:" << _vehicle_torque_setpoint.xyz[0] << " " << _vehicle_torque_setpoint.xyz[1] << " " << _vehicle_torque_setpoint.xyz[2] << std::endl;
#endif

	//Publish thrust and deflection of control surfaces
	_vehicle_thrust_setpoint.timestamp = hrt_absolute_time();
	_vehicle_thrust_setpoint.timestamp_sample = angular_velocity.timestamp_sample;
	_vehicle_thrust_setpoint_pub.publish(_vehicle_thrust_setpoint);

	_vehicle_torque_setpoint.timestamp = hrt_absolute_time();
	_vehicle_torque_setpoint.timestamp_sample = angular_velocity.timestamp_sample;
	_vehicle_torque_setpoint_pub.publish(_vehicle_torque_setpoint);


	if (_vcontrol_mode.flag_control_manual_enabled) {

		// Flaps control
		float flaps_control = 0.f; // default to no flaps

		/* map flaps by default to manual if valid */
		if (PX4_ISFINITE(_manual_control_setpoint.flaps)) {
			flaps_control = math::max(_manual_control_setpoint.flaps, 0.f); // do not consider negative switch settings
		}

		normalized_unsigned_setpoint_s flaps_setpoint;
		flaps_setpoint.timestamp = hrt_absolute_time();
		flaps_setpoint.normalized_setpoint = flaps_control;
		_flaps_setpoint_pub.publish(flaps_setpoint);

		// Spoilers control
		float spoilers_control = 0.f; // default to no spoilers

		switch (_param_fw_spoilers_man.get()) {
		case 0:
			break;

		case 1:
			// do not consider negative switch settings
			spoilers_control = PX4_ISFINITE(_manual_control_setpoint.flaps) ? math::max(_manual_control_setpoint.flaps, 0.f) : 0.f;
			break;

		case 2:
			// do not consider negative switch settings
			spoilers_control = PX4_ISFINITE(_manual_control_setpoint.aux1) ? math::max(_manual_control_setpoint.aux1, 0.f) : 0.f;
			break;
		}

		normalized_unsigned_setpoint_s spoilers_setpoint;
		spoilers_setpoint.timestamp = hrt_absolute_time();
		spoilers_setpoint.normalized_setpoint = spoilers_control;
		_spoilers_setpoint_pub.publish(spoilers_setpoint);
	}
}

void FixedwingRateControl::Run()
{

	if (should_exit()) {
		_vehicle_angular_velocity_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);

	// Update control switch param and store it to a variable
	_param_fw_ctrl.update();
	_ctrl_type = _param_fw_ctrl.get();

	if(!_ctrl_type){
		// only run controller if angular velocity changed
		if (_vehicle_angular_velocity_sub.updated() || (hrt_elapsed_time(&_last_run) > 20_ms)) { //TODO rate!

			// only update parameters if they changed
			bool params_updated = _parameter_update_sub.updated();

			// check for parameter updates
			if (params_updated) {
				// clear update
				parameter_update_s pupdate;
				_parameter_update_sub.copy(&pupdate);

				// update parameters from storage
				updateParams();
				parameters_update();
			}

			float dt = 0.f;

			static constexpr float DT_MIN = 0.002f;
			static constexpr float DT_MAX = 0.04f;

			vehicle_angular_velocity_s vehicle_angular_velocity{};

			if (_vehicle_angular_velocity_sub.copy(&vehicle_angular_velocity)) {
				dt = math::constrain((vehicle_angular_velocity.timestamp_sample - _last_run) * 1e-6f, DT_MIN, DT_MAX);
				_last_run = vehicle_angular_velocity.timestamp_sample;
			}

			if (dt < DT_MIN || dt > DT_MAX) {
				const hrt_abstime time_now_us = hrt_absolute_time();
				dt = math::constrain((time_now_us - _last_run) * 1e-6f, DT_MIN, DT_MAX);
				_last_run = time_now_us;
			}

			vehicle_angular_velocity_s angular_velocity{};
			_vehicle_angular_velocity_sub.copy(&angular_velocity);

			Vector3f rates(angular_velocity.xyz);
			Vector3f angular_accel{angular_velocity.xyz_derivative};

			// Tailsitter: rotate setpoint from hover to fixed-wing frame (controller is in fixed-wing frame, interface in hover)
			if (_vehicle_status.is_vtol_tailsitter) {
				rates = Vector3f(-angular_velocity.xyz[2], angular_velocity.xyz[1], angular_velocity.xyz[0]);
				angular_accel = Vector3f(-angular_velocity.xyz_derivative[2], angular_velocity.xyz_derivative[1],
							angular_velocity.xyz_derivative[0]);
			}

			// vehicle status update must be before the vehicle_control_mode poll, otherwise rate sp are not published during whole transition
			_vehicle_status_sub.update(&_vehicle_status);
			const bool is_in_transition_except_tailsitter = _vehicle_status.in_transition_mode
					&& !_vehicle_status.is_vtol_tailsitter;
			const bool is_fixed_wing = _vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING;
			_in_fw_or_transition_wo_tailsitter_transition =  is_fixed_wing || is_in_transition_except_tailsitter;

			_vehicle_control_mode_sub.update(&_vcontrol_mode);

			vehicle_land_detected_poll();

			vehicle_manual_poll();
			vehicle_land_detected_poll();

			/* if we are in rotary wing mode, do nothing */
			if (_vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING && !_vehicle_status.is_vtol) {
				perf_end(_loop_perf);
				return;
			}

			if (_vcontrol_mode.flag_control_rates_enabled) {

				const float airspeed = get_airspeed_and_update_scaling();

				/* reset integrals where needed */
				if (_rates_sp.reset_integral) {
					_rate_control.resetIntegral();
				}

				// Reset integrators if the aircraft is on ground or not in a state where the fw attitude controller is run
				if (_landed || !_in_fw_or_transition_wo_tailsitter_transition) {

					_rate_control.resetIntegral();
				}

				// Update saturation status from control allocation feedback
				// TODO: send the unallocated value directly for better anti-windup
				Vector3<bool> diffthr_enabled(
					_param_vt_fw_difthr_en & static_cast<int32_t>(VTOLFixedWingDifferentialThrustEnabledBit::ROLL_BIT),
					_param_vt_fw_difthr_en & static_cast<int32_t>(VTOLFixedWingDifferentialThrustEnabledBit::PITCH_BIT),
					_param_vt_fw_difthr_en & static_cast<int32_t>(VTOLFixedWingDifferentialThrustEnabledBit::YAW_BIT)
				);

				if (_vehicle_status.is_vtol_tailsitter) {
					// Swap roll and yaw
					diffthr_enabled.swapRows(0, 2);
				}

				// saturation handling for axis controlled by differential thrust (VTOL only)
				control_allocator_status_s control_allocator_status;

				// Set saturation flags for VTOL differential thrust feature
				// If differential thrust is enabled in an axis, assume it's the only torque authority and only update saturation using matrix 0 allocating the motors.
				if (_control_allocator_status_subs[0].update(&control_allocator_status)) {
					for (size_t i = 0; i < 3; i++) {
						if (diffthr_enabled(i)) {
							_rate_control.setPositiveSaturationFlag(i, control_allocator_status.unallocated_torque[i] > FLT_EPSILON);
							_rate_control.setNegativeSaturationFlag(i, control_allocator_status.unallocated_torque[i] < -FLT_EPSILON);
						}
					}
				}

				// Set saturation flags for control surface controlled axes
				if (_control_allocator_status_subs[_vehicle_status.is_vtol ? 1 : 0].update(&control_allocator_status)) {
					for (size_t i = 0; i < 3; i++) {
						if (!diffthr_enabled(i)) {
							_rate_control.setPositiveSaturationFlag(i, control_allocator_status.unallocated_torque[i] > FLT_EPSILON);
							_rate_control.setNegativeSaturationFlag(i, control_allocator_status.unallocated_torque[i] < -FLT_EPSILON);
						}
					}
				}

				/* bi-linear interpolation over airspeed for actuator trim scheduling */
				float trim_roll = _param_trim_roll.get();
				float trim_pitch = _param_trim_pitch.get();
				float trim_yaw = _param_trim_yaw.get();

				if (airspeed < _param_fw_airspd_trim.get()) {
					trim_roll += interpolate(airspeed, _param_fw_airspd_min.get(), _param_fw_airspd_trim.get(),
								_param_fw_dtrim_r_vmin.get(),
								0.0f);
					trim_pitch += interpolate(airspeed, _param_fw_airspd_min.get(), _param_fw_airspd_trim.get(),
								_param_fw_dtrim_p_vmin.get(),
								0.0f);
					trim_yaw += interpolate(airspeed, _param_fw_airspd_min.get(), _param_fw_airspd_trim.get(),
								_param_fw_dtrim_y_vmin.get(),
								0.0f);

				} else {
					trim_roll += interpolate(airspeed, _param_fw_airspd_trim.get(), _param_fw_airspd_max.get(), 0.0f,
								_param_fw_dtrim_r_vmax.get());
					trim_pitch += interpolate(airspeed, _param_fw_airspd_trim.get(), _param_fw_airspd_max.get(), 0.0f,
								_param_fw_dtrim_p_vmax.get());
					trim_yaw += interpolate(airspeed, _param_fw_airspd_trim.get(), _param_fw_airspd_max.get(), 0.0f,
								_param_fw_dtrim_y_vmax.get());
				}

				if (_vcontrol_mode.flag_control_rates_enabled) {
					_rates_sp_sub.update(&_rates_sp);

					Vector3f body_rates_setpoint = Vector3f(_rates_sp.roll, _rates_sp.pitch, _rates_sp.yaw);

					// Tailsitter: rotate setpoint from hover to fixed-wing frame (controller is in fixed-wing frame, interface in hover)
					if (_vehicle_status.is_vtol_tailsitter) {
						body_rates_setpoint = Vector3f(-_rates_sp.yaw, _rates_sp.pitch, _rates_sp.roll);
					}

					// Run attitude RATE controllers which need the desired attitudes from above, add trim.
					const Vector3f angular_acceleration_setpoint = _rate_control.update(rates, body_rates_setpoint, angular_accel, dt,
							_landed);

					const float roll_feedforward = _param_fw_rr_ff.get() * _airspeed_scaling * body_rates_setpoint(0);
					const float pitch_feedforward = _param_fw_pr_ff.get() * _airspeed_scaling * body_rates_setpoint(1);
					const float yaw_feedforward = _param_fw_yr_ff.get() * _airspeed_scaling * body_rates_setpoint(2);

					const float roll_u = angular_acceleration_setpoint(0) * _airspeed_scaling * _airspeed_scaling + roll_feedforward;
					const float pitch_u = angular_acceleration_setpoint(1) * _airspeed_scaling * _airspeed_scaling + pitch_feedforward;

					// Special case yaw in Acro: if the parameter FW_ACRO_YAW_CTL is not set then don't control yaw
					float yaw_u = 0.f;

					if (_vcontrol_mode.flag_control_attitude_enabled || _param_fw_acro_yaw_en.get()) {
						yaw_u = angular_acceleration_setpoint(2) * _airspeed_scaling * _airspeed_scaling + yaw_feedforward;

					} else {
						yaw_u = _manual_control_setpoint.yaw * _param_fw_man_y_sc.get();
						_rate_control.resetIntegral(2);
					}

					if (!PX4_ISFINITE(roll_u) || !PX4_ISFINITE(pitch_u) || !PX4_ISFINITE(yaw_u)) {
						_rate_control.resetIntegral();
					}

					_vehicle_torque_setpoint.xyz[0] = PX4_ISFINITE(roll_u) ? math::constrain(roll_u + trim_roll, -1.f, 1.f) : trim_roll;
					_vehicle_torque_setpoint.xyz[1] = PX4_ISFINITE(pitch_u) ? math::constrain(pitch_u + trim_pitch, -1.f, 1.f) : trim_pitch;
					_vehicle_torque_setpoint.xyz[2] = PX4_ISFINITE(yaw_u) ? math::constrain(yaw_u + trim_yaw, -1.f, 1.f) : trim_yaw;

					/* throttle passed through if it is finite */
					_vehicle_thrust_setpoint.xyz[0] = PX4_ISFINITE(_rates_sp.thrust_body[0]) ? _rates_sp.thrust_body[0] : 0.0f;

					/* scale effort by battery status */
					if (_param_fw_bat_scale_en.get() && _vehicle_thrust_setpoint.xyz[0] > 0.1f) {

						if (_battery_status_sub.updated()) {
							battery_status_s battery_status{};

							if (_battery_status_sub.copy(&battery_status) && battery_status.connected && battery_status.scale > 0.f) {
								_battery_scale = battery_status.scale;
							}
						}

						_vehicle_thrust_setpoint.xyz[0] *= _battery_scale;
					}
				}

				// publish rate controller status
				rate_ctrl_status_s rate_ctrl_status{};
				_rate_control.getRateControlStatus(rate_ctrl_status);
				rate_ctrl_status.timestamp = hrt_absolute_time();

				_rate_ctrl_status_pub.publish(rate_ctrl_status);

			} else {
				// full manual
				_rate_control.resetIntegral();
			}

			// Add feed-forward from roll control output to yaw control output
			// This can be used to counteract the adverse yaw effect when rolling the plane
			_vehicle_torque_setpoint.xyz[2] = math::constrain(_vehicle_torque_setpoint.xyz[2] + _param_fw_rll_to_yaw_ff.get() *
							_vehicle_torque_setpoint.xyz[0], -1.f, 1.f);

			// Tailsitter: rotate back to body frame from airspeed frame
			if (_vehicle_status.is_vtol_tailsitter) {
				const float helper = _vehicle_torque_setpoint.xyz[0];
				_vehicle_torque_setpoint.xyz[0] = _vehicle_torque_setpoint.xyz[2];
				_vehicle_torque_setpoint.xyz[2] = -helper;
			}



			/* Only publish if any of the proper modes are enabled */
			if (_vcontrol_mode.flag_control_rates_enabled ||
			_vcontrol_mode.flag_control_attitude_enabled ||
			_vcontrol_mode.flag_control_manual_enabled) {
				{

					_vehicle_thrust_setpoint.timestamp = hrt_absolute_time();
					_vehicle_thrust_setpoint.timestamp_sample = angular_velocity.timestamp_sample;
					_vehicle_thrust_setpoint_pub.publish(_vehicle_thrust_setpoint);

					_vehicle_torque_setpoint.timestamp = hrt_absolute_time();
					_vehicle_torque_setpoint.timestamp_sample = angular_velocity.timestamp_sample;
					_vehicle_torque_setpoint_pub.publish(_vehicle_torque_setpoint);
				}
			}

			updateActuatorControlsStatus(dt);

			// Manual flaps/spoilers control, also active in VTOL Hover. Is handled and published in FW Position controller/VTOL module if Auto.
			if (_vcontrol_mode.flag_control_manual_enabled) {

				// Flaps control
				float flaps_control = 0.f; // default to no flaps

				/* map flaps by default to manual if valid */
				if (PX4_ISFINITE(_manual_control_setpoint.flaps)) {
					flaps_control = math::max(_manual_control_setpoint.flaps, 0.f); // do not consider negative switch settings
				}

				normalized_unsigned_setpoint_s flaps_setpoint;
				flaps_setpoint.timestamp = hrt_absolute_time();
				flaps_setpoint.normalized_setpoint = flaps_control;
				_flaps_setpoint_pub.publish(flaps_setpoint);

				// Spoilers control
				float spoilers_control = 0.f; // default to no spoilers

				switch (_param_fw_spoilers_man.get()) {
				case 0:
					break;

				case 1:
					// do not consider negative switch settings
					spoilers_control = PX4_ISFINITE(_manual_control_setpoint.flaps) ? math::max(_manual_control_setpoint.flaps, 0.f) : 0.f;
					break;

				case 2:
					// do not consider negative switch settings
					spoilers_control = PX4_ISFINITE(_manual_control_setpoint.aux1) ? math::max(_manual_control_setpoint.aux1, 0.f) : 0.f;
					break;
				}

				normalized_unsigned_setpoint_s spoilers_setpoint;
				spoilers_setpoint.timestamp = hrt_absolute_time();
				spoilers_setpoint.normalized_setpoint = spoilers_control;
				_spoilers_setpoint_pub.publish(spoilers_setpoint);
			}
		}
	}

	else{
		if (_att_sub.updated() || (hrt_elapsed_time(&_last_run) > 20_ms))
			LQR();
	}
	// backup schedule
	ScheduleDelayed(20_ms);

	perf_end(_loop_perf);
}

void FixedwingRateControl::updateActuatorControlsStatus(float dt)
{
	for (int i = 0; i < 3; i++) {

		// We assume that the attitude is actuated by control surfaces
		// consuming power only when they move
		const float control_signal = _vehicle_torque_setpoint.xyz[i] - _control_prev[i];
		_control_prev[i] = _vehicle_torque_setpoint.xyz[i];

		_control_energy[i] += control_signal * control_signal * dt;
	}

	_energy_integration_time += dt;

	if (_energy_integration_time > 500e-3f) {

		actuator_controls_status_s status;
		status.timestamp = _vehicle_torque_setpoint.timestamp;

		for (int i = 0; i < 3; i++) {
			status.control_power[i] = _control_energy[i] / _energy_integration_time;
			_control_energy[i] = 0.f;
		}

		_actuator_controls_status_pub.publish(status);
		_energy_integration_time = 0.f;
	}
}

int FixedwingRateControl::task_spawn(int argc, char *argv[])
{
	bool vtol = false;

	if (argc > 1) {
		if (strcmp(argv[1], "vtol") == 0) {
			vtol = true;
		}
	}

	FixedwingRateControl *instance = new FixedwingRateControl(vtol);

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int FixedwingRateControl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int FixedwingRateControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
fw_rate_control is the fixed-wing rate controller.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("fw_rate_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_ARG("vtol", "VTOL mode", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int fw_rate_control_main(int argc, char *argv[])
{
	return FixedwingRateControl::main(argc, argv);
}
