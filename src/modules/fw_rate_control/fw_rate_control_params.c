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
 * AS IS AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
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
 * @file fw_rate_control_params.c
 *
 * Parameters defined by the fixed-wing rate control task
 *
 * @author Lorenz Meier <lorenz@px4.io>
 * @author Thomas Gubler <thomas@px4.io>
 */

/**
 * Use airspeed for control
 *
 * If set to 1, the airspeed measurement data, if valid, is used in the following controllers:
 * - Rate controller: output scaling
 * - Attitude controller: coordinated turn controller
 * - Position controller: airspeed setpoint tracking, takeoff logic
 * - VTOL: transition logic
 *
 * @boolean
 * @group FW Rate Control
 */
PARAM_DEFINE_INT32(FW_USE_AIRSPD, 1);

/**
 * Pitch rate proportional gain.
 *
 * @unit %/rad/s
 * @min 0.0
 * @max 10
 * @decimal 3
 * @increment 0.005
 * @group FW Rate Control
 */
PARAM_DEFINE_FLOAT(FW_PR_P, 0.08f);

/**
 * Pitch rate derivative gain.
 *
 * Pitch rate differential gain.
 *
 * @unit %/rad/s
 * @min 0.0
 * @max 10
 * @decimal 3
 * @increment 0.005
 * @group FW Rate Control
 */
PARAM_DEFINE_FLOAT(FW_PR_D, 0.f);

/**
 * Pitch rate integrator gain.
 *
 * @unit %/rad
 * @min 0.0
 * @max 10
 * @decimal 3
 * @increment 0.005
 * @group FW Rate Control
 */
PARAM_DEFINE_FLOAT(FW_PR_I, 0.1f);

/**
 * Pitch rate integrator limit
 *
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.05
 * @group FW Rate Control
 */
PARAM_DEFINE_FLOAT(FW_PR_IMAX, 0.4f);

/**
 * Roll rate proportional gain
 *
 * @unit %/rad/s
 * @min 0.0
 * @max 10
 * @decimal 3
 * @increment 0.005
 * @group FW Rate Control
 */
PARAM_DEFINE_FLOAT(FW_RR_P, 0.05f);

/**
 * Roll rate derivative gain
 *
 * @unit %/rad/s
 * @min 0.0
 * @max 10
 * @decimal 3
 * @increment 0.005
 * @group FW Rate Control
 */
PARAM_DEFINE_FLOAT(FW_RR_D, 0.0f);

/**
 * Roll rate integrator gain
 *
 * @unit %/rad
 * @min 0.0
 * @max 10
 * @decimal 2
 * @increment 0.01
 * @group FW Rate Control
 */
PARAM_DEFINE_FLOAT(FW_RR_I, 0.1f);

/**
 * Roll integrator limit
 *
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.05
 * @group FW Rate Control
 */
PARAM_DEFINE_FLOAT(FW_RR_IMAX, 0.2f);

/**
 * Yaw rate proportional gain
 *
 * @unit %/rad/s
 * @min 0.0
 * @max 10
 * @decimal 3
 * @increment 0.005
 * @group FW Rate Control
 */
PARAM_DEFINE_FLOAT(FW_YR_P, 0.05f);

/**
 * Yaw rate derivative gain
 *
 * @unit %/rad/s
 * @min 0.0
 * @max 10
 * @decimal 3
 * @increment 0.005
 * @group FW Rate Control
 */
PARAM_DEFINE_FLOAT(FW_YR_D, 0.0f);

/**
 * Yaw rate integrator gain
 *
 * @unit %/rad
 * @min 0.0
 * @max 10
 * @decimal 1
 * @increment 0.5
 * @group FW Rate Control
 */
PARAM_DEFINE_FLOAT(FW_YR_I, 0.1f);

/**
 * Yaw rate integrator limit
 *
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.05
 * @group FW Rate Control
 */
PARAM_DEFINE_FLOAT(FW_YR_IMAX, 0.2f);

/**
 * Roll rate feed forward
 *
 * Direct feed forward from rate setpoint to control surface output.
 *
 * @unit %/rad/s
 * @min 0.0
 * @max 10.0
 * @decimal 2
 * @increment 0.05
 * @group FW Rate Control
 */
PARAM_DEFINE_FLOAT(FW_RR_FF, 0.5f);

/**
 * Pitch rate feed forward
 *
 * Direct feed forward from rate setpoint to control surface output
 *
 * @unit %/rad/s
 * @min 0.0
 * @max 10.0
 * @decimal 2
 * @increment 0.05
 * @group FW Rate Control
 */
PARAM_DEFINE_FLOAT(FW_PR_FF, 0.5f);

/**
 * Yaw rate feed forward
 *
 * Direct feed forward from rate setpoint to control surface output
 *
 * @unit %/rad/s
 * @min 0.0
 * @max 10.0
 * @decimal 2
 * @increment 0.05
 * @group FW Rate Control
 */
PARAM_DEFINE_FLOAT(FW_YR_FF, 0.3f);

/**
 * Acro body roll max rate setpoint
 *
 * @min 10
 * @max 720
 * @unit deg
 * @group FW Rate Control
 */
PARAM_DEFINE_FLOAT(FW_ACRO_X_MAX, 90);

/**
 * Acro body pitch max rate setpoint
 *
 * @min 10
 * @max 720
 * @unit deg
 * @group FW Rate Control
 */
PARAM_DEFINE_FLOAT(FW_ACRO_Y_MAX, 90);

/**
 * Acro body yaw max rate setpoint
 *
 * @min 10
 * @max 720
 * @unit deg
 * @group FW Rate Control
 */
PARAM_DEFINE_FLOAT(FW_ACRO_Z_MAX, 45);

/**
 * Enable throttle scale by battery level
 *
 * This compensates for voltage drop of the battery over time by attempting to
 * normalize performance across the operating range of the battery.
 *
 * @boolean
 * @group FW Rate Control
 */
PARAM_DEFINE_INT32(FW_BAT_SCALE_EN, 0);

/**
 * Enable airspeed scaling
 *
 * This enables a logic that automatically adjusts the output of the rate controller to take
 * into account the real torque produced by an aerodynamic control surface given
 * the current deviation from the trim airspeed (FW_AIRSPD_TRIM).
 *
 * Enable when using aerodynamic control surfaces (e.g.: plane)
 * Disable when using rotor wings (e.g.: autogyro)
 *
 * @boolean
 * @group FW Rate Control
 */
PARAM_DEFINE_INT32(FW_ARSP_SCALE_EN, 1);

/**
* Roll trim increment at minimum airspeed
*
* This increment is added to TRIM_ROLL when airspeed is FW_AIRSPD_MIN.
 *
 * @group FW Rate Control
 * @min -0.5
 * @max 0.5
 * @decimal 2
 * @increment 0.01
 */
PARAM_DEFINE_FLOAT(FW_DTRIM_R_VMIN, 0.0f);

/**
* Pitch trim increment at minimum airspeed
*
* This increment is added to TRIM_PITCH when airspeed is FW_AIRSPD_MIN.
 *
 * @group FW Rate Control
 * @min -0.5
 * @max 0.5
 * @decimal 2
 * @increment 0.01
 */
PARAM_DEFINE_FLOAT(FW_DTRIM_P_VMIN, 0.0f);

/**
* Yaw trim increment at minimum airspeed
*
* This increment is added to TRIM_YAW when airspeed is FW_AIRSPD_MIN.
 *
 * @group FW Rate Control
 * @min -0.5
 * @max 0.5
 * @decimal 2
 * @increment 0.01
 */
PARAM_DEFINE_FLOAT(FW_DTRIM_Y_VMIN, 0.0f);

/**
* Roll trim increment at maximum airspeed
*
* This increment is added to TRIM_ROLL when airspeed is FW_AIRSPD_MAX.
 *
 * @group FW Rate Control
 * @min -0.5
 * @max 0.5
 * @decimal 2
 * @increment 0.01
 */
PARAM_DEFINE_FLOAT(FW_DTRIM_R_VMAX, 0.0f);

/**
* Pitch trim increment at maximum airspeed
*
* This increment is added to TRIM_PITCH when airspeed is FW_AIRSPD_MAX.
 *
 * @group FW Rate Control
 * @min -0.5
 * @max 0.5
 * @decimal 2
 * @increment 0.01
 */
PARAM_DEFINE_FLOAT(FW_DTRIM_P_VMAX, 0.0f);

/**
* Yaw trim increment at maximum airspeed
*
* This increment is added to TRIM_YAW when airspeed is FW_AIRSPD_MAX.
 *
 * @group FW Rate Control
 * @min -0.5
 * @max 0.5
 * @decimal 2
 * @increment 0.01
 */
PARAM_DEFINE_FLOAT(FW_DTRIM_Y_VMAX, 0.0f);

/**
 * Manual roll scale
 *
 * Scale factor applied to the desired roll actuator command in full manual mode. This parameter allows
 * to adjust the throws of the control surfaces.
 *
 * @unit norm
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.01
 * @group FW Rate Control
 */
PARAM_DEFINE_FLOAT(FW_MAN_R_SC, 1.0f);

/**
 * Manual pitch scale
 *
 * Scale factor applied to the desired pitch actuator command in full manual mode. This parameter allows
 * to adjust the throws of the control surfaces.
 *
 * @unit norm
 * @min 0.0
 * @decimal 2
 * @increment 0.01
 * @group FW Rate Control
 */
PARAM_DEFINE_FLOAT(FW_MAN_P_SC, 1.0f);

/**
 * Manual yaw scale
 *
 * Scale factor applied to the desired yaw actuator command in full manual mode. This parameter allows
 * to adjust the throws of the control surfaces.
 *
 * @unit norm
 * @min 0.0
 * @decimal 2
 * @increment 0.01
 * @group FW Rate Control
 */
PARAM_DEFINE_FLOAT(FW_MAN_Y_SC, 1.0f);

/**
 * Roll control to yaw control feedforward gain.
 *
 * This gain can be used to counteract the "adverse yaw" effect for fixed wings.
 * When the plane enters a roll it will tend to yaw the nose out of the turn.
 * This gain enables the use of a yaw actuator to counteract this effect.
 *
 * @min 0.0
 * @decimal 1
 * @increment 0.01
 * @group FW Rate Control
 */
PARAM_DEFINE_FLOAT(FW_RLL_TO_YAW_FF, 0.0f);

/**
 * Spoiler input in manual flight
 *
 * Chose source for manual setting of spoilers in manual flight modes.
 *
 * @value 0 Disabled
 * @value 1 Flaps channel
 * @value 2 Aux1
 * @group FW Rate Control
 */
PARAM_DEFINE_INT32(FW_SPOILERS_MAN, 0);

/**
 * Enable yaw rate controller in Acro
 *
 * If this parameter is set to 1, the yaw rate controller is enabled in Fixed-wing Acro mode.
 * Otherwise the pilot commands directly the yaw actuator.
 * It is disabled by default because an active yaw rate controller will fight against the
 * natural turn coordination of the plane.
 *
 * @boolean
 * @group FW Rate Control
 */
PARAM_DEFINE_INT32(FW_ACRO_YAW_EN, 0);


/**
 * Attitude Controller Type
 *
 * Makes it possible to select between different attitude controllers.
 *
 * @group FW LQR Control
 * @value 0 Default PID
 * @value 1 Experimental Control
 */
PARAM_DEFINE_INT32(FW_CTRL, 0);


/**
 * LQR LATERAL K 11 parameter
 *
 * Sets the 11 parameter of the LQR LATERAL K.
 *
 * @group FW LQR Control
 */
PARAM_DEFINE_FLOAT(FW_LQR_LAT_K11, 0.110347097410768f);

/**
 * LQR LATERAL K 12 parameter
 *
 * Sets the 12 parameter of the LQR LATERAL K.
 *
 * @group FW LQR Control
 */
PARAM_DEFINE_FLOAT(FW_LQR_LAT_K12, 0.133194688173106f);

/**
 * LQR LATERAL K 13 parameter
 *
 * Sets the 13 parameter of the LQR LATERAL K.
 *
 * @group FW LQR Control
 */
PARAM_DEFINE_FLOAT(FW_LQR_LAT_K13, 1.485121132708253f);

/**
 * LQR LATERAL K 14 parameter
 *
 * Sets the 14 parameter of the LQR LATERAL K.
 *
 * @group FW LQR Control
 */
PARAM_DEFINE_FLOAT(FW_LQR_LAT_K14, -1.414125443178853f);

/**
 * LQR LATERAL K 21 parameter
 *
 * Sets the 21 parameter of the LQR LATERAL K.
 *
 * @group FW LQR Control
 */
PARAM_DEFINE_FLOAT(FW_LQR_LAT_K21, 1.553387970828113e-05f);

/**
 * LQR LATERAL K 22 parameter
 *
 * Sets the 22 parameter of the LQR LATERAL K.
 *
 * @group FW LQR Control
 */
PARAM_DEFINE_FLOAT(FW_LQR_LAT_K22, 0.030935344339421f);

/**
 * LQR LATERAL K 23 parameter
 *
 * Sets the 23 parameter of the LQR LATERAL K.
 *
 * @group FW LQR Control
 */
PARAM_DEFINE_FLOAT(FW_LQR_LAT_K23, 0.024804921892870f);


/**
 * LQR LATERAL K 24 parameter
 *
 * Sets the 24 parameter of the LQR LATERAL K.
 *
 * @group FW LQR Control
 */
PARAM_DEFINE_FLOAT(FW_LQR_LAT_K24, 0.015787050206141f);



/**
 * LQR LONGITUDINAL K 1 parameter
 *
 * Sets the 1 parameter of the LQR K.
 *
 * @group FW LQR Control
 */
PARAM_DEFINE_FLOAT(FW_LQR_LON_K1, 0.166013197475164f);

/**
 * LQR LONGITUDINAL K 2 parameter
 *
 * Sets the 2 parameter of the LQR K.
 *
 * @group FW LQR Control
 */
PARAM_DEFINE_FLOAT(FW_LQR_LON_K2, 0.651196946781365f);

/**
 * LQR LONGITUDINAL K 3 parameter
 *
 * Sets the 3 parameter of the LQR K.
 *
 * @group FW LQR Control
 */
PARAM_DEFINE_FLOAT(FW_LQR_LON_K3, -0.500000000000001f);



/**
 * LQR deltaE trim value
 *
 * Sets the trim value for the elevator deflection.
 *
 * @group FW LQR Control
 */
PARAM_DEFINE_FLOAT(FW_LQR_ELEV_TR, 0.051054634516116f);



/**
 * LQR time value
 *
 * Sets the inverse of the frequency at which the LQR algorithm is computed.
 *
 * @group FW LQR Control
 */
PARAM_DEFINE_FLOAT(FW_LQR_TS, 0.004f);


/**
 * Maximum rudder deflection
 *
 * Sets the maximum rudder deflection for antiwindup.
 *
 * @group FW LQR Control
 */
PARAM_DEFINE_FLOAT(FW_LQR_RUD_MAX, 1f);

/**
 * Minimum rudder deflection
 *
 * Sets the minimum rudder deflection for antiwindup.
 *
 * @group FW LQR Control
 */
PARAM_DEFINE_FLOAT(FW_LQR_RUD_MIN, -1f);



/**
 * LQR theta trim value
 *
 * Sets the trim value for pitch.
 *
 * @group FW LQR Control
 */
PARAM_DEFINE_FLOAT(FW_LQR_THETA_TR, 0.083054257670405f);


