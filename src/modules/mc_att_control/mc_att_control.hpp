/****************************************************************************
 *
 *   Copyright (c) 2013-2025 PX4 Development Team. All rights reserved.
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

#pragma once

#include <matrix/matrix/math.hpp>
#include <perf/perf_counter.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/WorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/autotune_attitude_control_status.h>
#include <uORB/topics/hover_thrust_estimate.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
// #include <uORB/topics/vehicle_torque_setpoint.h>
// #include <uORB/topics/vehicle_thrust_setpoint.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <lib/mathlib/math/filter/AlphaFilter.hpp>
#include <lib/slew_rate/SlewRate.hpp>
#include <modules/flight_mode_manager/tasks/Utility/StickYaw.hpp>

#include <AttitudeControl.hpp>

using namespace time_literals;

class MulticopterAttitudeControl :
	public ModuleBase<MulticopterAttitudeControl>,
	public ModuleParams,
	public px4::WorkItem
{
public:
	MulticopterAttitudeControl(bool vtol = false);
	~MulticopterAttitudeControl() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();

private:
	void Run() override;

	/**
	 * initialize some vectors/matrices from parameters
	 */
	void parameters_updated();
	void reset_mpc_state();
	void control_attitude_rates(float dt, const matrix::Vector3f &rates,  const matrix::Quatf &q);
	float throttle_curve(float throttle_stick_input);

	/**
	 * Generate & publish an attitude setpoint from stick inputs
	 */
	void generate_attitude_setpoint(const matrix::Quatf &q, float dt);

	AttitudeControl _attitude_control; /**< class for attitude control calculations */
	StickYaw _stick_yaw{this};

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	uORB::Subscription _hover_thrust_estimate_sub{ORB_ID(hover_thrust_estimate)};
	uORB::Subscription _autotune_attitude_control_status_sub{ORB_ID(autotune_attitude_control_status)};
	uORB::Subscription _manual_control_setpoint_sub{ORB_ID(manual_control_setpoint)};
	uORB::Subscription _vehicle_attitude_setpoint_sub{ORB_ID(vehicle_attitude_setpoint)};
	uORB::Subscription _vehicle_control_mode_sub{ORB_ID(vehicle_control_mode)};
	uORB::Subscription _vehicle_land_detected_sub{ORB_ID(vehicle_land_detected)};
	uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _vehicle_angular_velocity_sub{ORB_ID(vehicle_angular_velocity)};

	uORB::SubscriptionCallbackWorkItem _vehicle_attitude_sub{this, ORB_ID(vehicle_attitude)};

	uORB::Publication<vehicle_rates_setpoint_s>    _vehicle_rates_setpoint_pub{ORB_ID(vehicle_rates_setpoint)};
	// uORB::Publication<vehicle_torque_setpoint_s>  _vehicle_torque_setpoint_pub{ORB_ID(vehicle_torque_setpoint)};
	// uORB::Publication<vehicle_thrust_setpoint_s>  _vehicle_thrust_setpoint_pub{ORB_ID(vehicle_thrust_setpoint)};
	uORB::Publication<vehicle_attitude_setpoint_s> _vehicle_attitude_setpoint_pub;

	manual_control_setpoint_s _manual_control_setpoint{};
	vehicle_control_mode_s    _vehicle_control_mode{};

	perf_counter_t _loop_perf;

	matrix::Vector3f _thrust_setpoint_body;

	float _man_yaw_sp{0.f};
	float _man_tilt_max;

	float _hover_thrust_estimate{NAN};
	SlewRate<float> _hover_thrust_slew_rate{};
	SlewRate<float> _manual_throttle_minimum{};
	SlewRate<float> _manual_throttle_maximum{};
	AlphaFilter<float> _man_roll_input_filter;
	AlphaFilter<float> _man_pitch_input_filter;

	hrt_abstime _last_run{0};
	hrt_abstime _last_attitude_setpoint{0};

	bool _heading_good_for_control{true};
	bool _spooled_up{false};
	bool _landed{true};
	bool _vehicle_type_rotary_wing{true};
	bool _vtol{false};
	bool _vtol_tailsitter{false};
	bool _vtol_in_transition_mode{false};
	bool _last_armed_state{false};  // tracks true armed state for disarm detection
	bool _mpc_initialized{false};

	uint8_t _quat_reset_counter{0};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::MC_AIRMODE>)         _param_mc_airmode,
		(ParamFloat<px4::params::MC_MAN_TILT_TAU>)  _param_mc_man_tilt_tau,

		(ParamFloat<px4::params::MC_ROLL_P>)        _param_mc_roll_p,
		(ParamFloat<px4::params::MC_PITCH_P>)       _param_mc_pitch_p,
		(ParamFloat<px4::params::MC_YAW_P>)         _param_mc_yaw_p,
		(ParamFloat<px4::params::MC_YAW_WEIGHT>)    _param_mc_yaw_weight,

		(ParamFloat<px4::params::MC_ROLLRATE_MAX>)  _param_mc_rollrate_max,
		(ParamFloat<px4::params::MC_PITCHRATE_MAX>) _param_mc_pitchrate_max,
		(ParamFloat<px4::params::MC_YAWRATE_MAX>)   _param_mc_yawrate_max,

		(ParamFloat<px4::params::MAN_DEADZONE>)      _param_man_deadzone,
		(ParamFloat<px4::params::MPC_MAN_TILT_MAX>)  _param_mpc_man_tilt_max,
		(ParamFloat<px4::params::MPC_MANTHR_MIN>)    _param_mpc_manthr_min,
		(ParamFloat<px4::params::MPC_THR_MAX>)       _param_mpc_thr_max,
		(ParamFloat<px4::params::MPC_THR_HOVER>)     _param_mpc_thr_hover,
		(ParamInt<px4::params::MPC_THR_CURVE>)       _param_mpc_thr_curve,

		(ParamFloat<px4::params::COM_SPOOLUP_TIME>)  _param_com_spoolup_time
	)

	void QPhild();

	// ----- Constant matrices (computed once in constructor) -----
	static matrix::SquareMatrix<float, 6>  _E_inv;   // (2*H'H + W)^-1,  6x6
	static matrix::Matrix<float, 6, 24>    _CC_trans; // CC transposed,    6x24
	static matrix::SquareMatrix<float, 24> _T;        // CC*E_inv*CC',    24x24  ← pre-computed ONCE
	static matrix::Matrix<float, 24, 6>    _CC;       // constraint LHS,  24x6
	static matrix::Matrix<float, 24, 1>    _dd;       // constraint RHS,  24x1
	static matrix::Matrix<float, 24, 3>    _dupast;   // delta-u history, 24x3
	static matrix::Matrix<float, 15, 9>    _P;        // prediction,      15x9
	static matrix::Matrix<float, 15, 6>    _H;        // Hessian factor,  15x6
	static matrix::Matrix<float, 6, 15>    _H_trans;  // H transposed,    6x15
	static matrix::SquareMatrix<float, 6>  _W;        // weight matrix,   6x6

	// ----- Per-cycle working vectors (small, safe on BSS) -----
	static matrix::Vector<float, 6>  _tempEF;
	static matrix::Vector<float, 24> _Kvec;
	static matrix::Vector<float, 24> _lambda;
	static matrix::Vector<float, 24> _lambda_prev;
	static matrix::Vector<float, 6>  _tempVec6;
	static matrix::Vector<float, 6>  _tempV6_2;
	static matrix::Vector<float, 6>  _DeltaU;
	static matrix::Vector<float, 6>  _F;
	static matrix::Vector<float, 24> _d_local;

	// ----- State / control (persistent across cycles) -----
	static matrix::Matrix<float, 6, 1> _x;         // full state [phi p theta q psi r]
	static matrix::Matrix<float, 3, 1> _u;         // current control input
	static matrix::Matrix<float, 9, 1> _Xf;        // augmented state for MPC
	static matrix::Matrix<float, 3, 1> _des;       // desired rate setpoint (MPC input)

	// ----- System matrices (rebuilt each cycle only for Ad) -----
	static matrix::Matrix<float, 6, 6> _Ad;        // discretised A  (updates with dt)
	static matrix::Matrix<float, 6, 3> _Bd;        // discretised B  (constant, set in constructor)
	static matrix::Matrix<float, 3, 6> _Cd;        // output matrix  (constant, set in constructor)

	// ----- Temporary computation matrices -----
	static matrix::Matrix<float, 15, 1> _temp15;
	static matrix::Matrix<float, 6,  1> _temp6;
	static matrix::Matrix<float, 3,  1> _delta_first;
	static matrix::Matrix<float, 6,  1> _x_diff;
	static matrix::Matrix<float, 3,  1> _y_out;    // renamed from y (avoids ambiguity)
	static matrix::Matrix<float, 15, 3> _Rs;
	static matrix::Matrix<float, 6, 1> _x_prev;

	matrix::Vector3f _rates_sp;      // rate setpoint from attitude controller
	matrix::Vector3f _att_control;   // MPC output (normalised to [-1,1])
};

