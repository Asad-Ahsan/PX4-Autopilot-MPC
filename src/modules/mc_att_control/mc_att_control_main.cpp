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

/**
 * @file mc_att_control_main.cpp
 * Multicopter attitude controller.
 */

#include <matrix/matrix/math.hpp>
#include "mc_att_control.hpp"
#include <drivers/drv_hrt.h>
#include <mathlib/math/Limits.hpp>
#include <mathlib/math/Functions.hpp>
#include "AttitudeControl/AttitudeControlMath.hpp"
#include <cmath>

using namespace matrix;

// Constant matrices (computed once, never change)
matrix::SquareMatrix<float, 6>  MulticopterAttitudeControl::_E_inv{};
matrix::Matrix<float, 6, 24>    MulticopterAttitudeControl::_CC_trans{};
matrix::SquareMatrix<float, 24> MulticopterAttitudeControl::_T{};
matrix::Matrix<float, 24, 6>    MulticopterAttitudeControl::_CC{};
matrix::Matrix<float, 24, 1>    MulticopterAttitudeControl::_dd{};
matrix::Matrix<float, 24, 3>    MulticopterAttitudeControl::_dupast{};
matrix::Matrix<float, 15, 9>    MulticopterAttitudeControl::_P{};
matrix::Matrix<float, 15, 6>    MulticopterAttitudeControl::_H{};
matrix::Matrix<float, 6, 15>    MulticopterAttitudeControl::_H_trans{};
matrix::SquareMatrix<float, 6>  MulticopterAttitudeControl::_W{};

// Working vectors
matrix::Vector<float, 6>  MulticopterAttitudeControl::_tempEF{};
matrix::Vector<float, 24> MulticopterAttitudeControl::_Kvec{};
matrix::Vector<float, 24> MulticopterAttitudeControl::_lambda{};
matrix::Vector<float, 24> MulticopterAttitudeControl::_lambda_prev{};
matrix::Vector<float, 6>  MulticopterAttitudeControl::_tempVec6{};
matrix::Vector<float, 6>  MulticopterAttitudeControl::_tempV6_2{};
matrix::Vector<float, 6>  MulticopterAttitudeControl::_DeltaU{};
matrix::Vector<float, 6>  MulticopterAttitudeControl::_F{};
matrix::Vector<float, 24> MulticopterAttitudeControl::_d_local{};

// State / control
matrix::Matrix<float, 6, 1> MulticopterAttitudeControl::_x{};
matrix::Matrix<float, 3, 1> MulticopterAttitudeControl::_u{};
matrix::Matrix<float, 9, 1> MulticopterAttitudeControl::_Xf{};
matrix::Matrix<float, 3, 1> MulticopterAttitudeControl::_des{};

// System matrices
matrix::Matrix<float, 6, 6> MulticopterAttitudeControl::_Ad{};
matrix::Matrix<float, 6, 3> MulticopterAttitudeControl::_Bd{};
matrix::Matrix<float, 3, 6> MulticopterAttitudeControl::_Cd{};

// Temporaries
matrix::Matrix<float, 15, 1> MulticopterAttitudeControl::_temp15{};
matrix::Matrix<float, 6,  1> MulticopterAttitudeControl::_temp6{};
matrix::Matrix<float, 3,  1> MulticopterAttitudeControl::_delta_first{};
matrix::Matrix<float, 6,  1> MulticopterAttitudeControl::_x_diff{};
matrix::Matrix<float, 3,  1> MulticopterAttitudeControl::_y_out{};
matrix::Matrix<float, 15, 3> MulticopterAttitudeControl::_Rs{};
matrix::Matrix<float, 6, 1> MulticopterAttitudeControl::_x_prev{};

// =============================================================================
// CONSTRUCTOR
// =============================================================================

MulticopterAttitudeControl::MulticopterAttitudeControl(bool vtol) :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers),
	_vehicle_attitude_setpoint_pub(vtol ? ORB_ID(mc_virtual_attitude_setpoint) : ORB_ID(vehicle_attitude_setpoint)),
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")),
	_vtol(vtol)
{
	parameters_updated();

	_manual_throttle_minimum.setSlewRate(0.05f);
	_manual_throttle_minimum.update(0.0f, 0.0f);
	_manual_throttle_maximum.setSlewRate(0.5f);
	_manual_throttle_maximum.update(0.0f, 0.0f);
	_hover_thrust_slew_rate.setSlewRate(0.05f);
	_hover_thrust_slew_rate.update(_param_mpc_thr_hover.get(), 0.0f);

	// =========================================================================
	// MPC ONE-TIME INITIALISATION
	// Everything here is computed exactly once at startup.
	// =========================================================================

	reset_mpc_state();
	_rates_sp.zero();
	_att_control.zero();

	// ----- H matrix (15x6) -----
	static const float H_data[15][6] = {
		{81.1688f,0,0,0,0,0},
		{0,63.1712f,0,0,0,0},
		{0,0,47.3934f,0,0,0},
		{162.3377f,0,0,81.1688f,0,0},
		{0,126.3424f,0,0,63.1712f,0},
		{0,0,94.7867f,0,0,47.3934f},
		{243.5065f,0,0,162.3377f,0,0},
		{0,189.5136f,0,0,126.3424f,0},
		{0,0,142.1801f,0,0,94.7867f},
		{324.6753f,0,0,243.5065f,0,0},
		{0,252.6848f,0,0,189.5136f,0},
		{0,0,189.5735f,0,0,142.1801f},
		{405.8442f,0,0,324.6753f,0,0},
		{0,315.8560f,0,0,252.6848f,0},
		{0,0,236.9668f,0,0,189.5735f}
	};
	_H.zero();
	for (int i = 0; i < 15; i++)
		for (int j = 0; j < 6; j++)
			_H(i,j) = H_data[i][j];

	// ----- P matrix (15x9) -----
	static const float P_data[15][9] = {
		{0,0,0,0,0,0,1,0,0},
		{0,0,0,0,0,0,0,1,0},
		{0,0,0,0,0,0,0,0,1},
		{0,1,0,0,0,0,1,0,0},
		{0,0,0,1,0,0,0,1,0},
		{0,0,0,0,0,1,0,0,1},
		{0,2,0,0,0,0,1,0,0},
		{0,0,0,2,0,0,0,1,0},
		{0,0,0,0,0,2,0,0,1},
		{0,3,0,0,0,0,1,0,0},
		{0,0,0,3,0,0,0,1,0},
		{0,0,0,0,0,3,0,0,1},
		{0,4,0,0,0,0,1,0,0},
		{0,0,0,4,0,0,0,1,0},
		{0,0,0,0,0,4,0,0,1}
	};
	_P.zero();
	for (int i = 0; i < 15; i++)
		for (int j = 0; j < 9; j++)
			_P(i,j) = P_data[i][j];

	// ----- W matrix (6x6) -----
	_W.zero();
	_W(0,0) = 0.075f * 0.5f;
	_W(1,1) = 0.075f * 0.5f;
	_W(2,2) = 0.045f * 0.5f;
	_W(3,3) = 0.075f * 0.5f;
	_W(4,4) = 0.075f * 0.5f;
	_W(5,5) = 0.045f * 0.5f;

	// ----- E = 2*(H'*H + W),  E_inv = E^{-1} -----
	_H_trans = _H.transpose();
	SquareMatrix<float,6> E = (SquareMatrix<float,6>)(_H_trans * _H);
	E += _W;
	E *= 2.0f;
	_E_inv = E.I();

	// ----- CC matrix (24x6) -----
	static const float CC_data[24][6] = {
		{ 1, 0, 0, 0, 0, 0}, { 0, 1, 0, 0, 0, 0}, { 0, 0, 1, 0, 0, 0},
		{ 0, 0, 0, 1, 0, 0}, { 0, 0, 0, 0, 1, 0}, { 0, 0, 0, 0, 0, 1},
		{-1, 0, 0, 0, 0, 0}, { 0,-1, 0, 0, 0, 0}, { 0, 0,-1, 0, 0, 0},
		{ 0, 0, 0,-1, 0, 0}, { 0, 0, 0, 0,-1, 0}, { 0, 0, 0, 0, 0,-1},
		{ 1, 0, 0, 0, 0, 0}, { 0, 1, 0, 0, 0, 0}, { 0, 0, 1, 0, 0, 0},
		{ 1, 0, 0, 1, 0, 0}, { 0, 1, 0, 0, 1, 0}, { 0, 0, 1, 0, 0, 1},
		{-1, 0, 0, 0, 0, 0}, { 0,-1, 0, 0, 0, 0}, { 0, 0,-1, 0, 0, 0},
		{-1, 0, 0,-1, 0, 0}, { 0,-1, 0, 0,-1, 0}, { 0, 0,-1, 0, 0,-1}
	};
	_CC.zero();
	for (int i = 0; i < 24; i++)
		for (int j = 0; j < 6; j++)
			_CC(i,j) = CC_data[i][j];

	_CC_trans = _CC.transpose();

	// ----- dd vector (24x1) -----
	static const float dd_data[24] = {
		0.4796f,0.4796f,0.1161f, 0.4796f,0.4796f,0.1161f,
		0.4796f,0.4796f,0.1161f, 0.4796f,0.4796f,0.1161f,
		0.7794f,0.7794f,0.1935f, 0.7794f,0.7794f,0.1935f,
		0.7794f,0.7794f,0.1935f, 0.7794f,0.7794f,0.1935f
	};
	_dd.zero();
	for (int i = 0; i < 24; i++) _dd(i,0) = dd_data[i];

	// ----- dupast matrix (24x3) -----
	static const float dupast_data[24][3] = {
		{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},
		{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},
		{-1,0,0},{0,-1,0},{0,0,-1},
		{-1,0,0},{0,-1,0},{0,0,-1},
		{ 1,0,0},{0, 1,0},{0,0, 1},
		{ 1,0,0},{0, 1,0},{0,0, 1}
	};
	_dupast.zero();
	for (int i = 0; i < 24; i++)
		for (int j = 0; j < 3; j++)
			_dupast(i,j) = dupast_data[i][j];

	_T.zero();
	for (int i = 0; i < 24; i++) {
		for (int j = 0; j < 24; j++) {
			float sum = 0.f;
			for (int k = 0; k < 6; k++)
				for (int m = 0; m < 6; m++)
					sum += _CC(i,k) * _E_inv(k,m) * _CC(j,m);
			_T(i,j) = sum;
		}
	}

	_Bd.zero();
	_Bd(0,0) = 0.0032468f; _Bd(1,0) = 1.62338f;
	_Bd(2,1) = 0.0025268f; _Bd(3,1) = 1.26342f;
	_Bd(4,2) = 0.0018957f; _Bd(5,2) = 0.94786f;

	_Cd.zero();
	_Cd(0,1) = 1.f;
	_Cd(1,3) = 1.f;
	_Cd(2,5) = 1.f;

	// Rs is also constant (block-diagonal identity stacked 5 times)
	_Rs.zero();
	for (int i = 0; i < 5; i++) {
		_Rs(i*3+0, 0) = 1.f;
		_Rs(i*3+1, 1) = 1.f;
		_Rs(i*3+2, 2) = 1.f;
	}
	_mpc_initialized = true;
	PX4_INFO("MPC controller initialised");
}

MulticopterAttitudeControl::~MulticopterAttitudeControl()
{
	perf_free(_loop_perf);
}

void MulticopterAttitudeControl::reset_mpc_state()
{
	_x.zero();
	_u.zero();
	_Xf.zero();
	_x_prev.zero();
	_des.zero();
	_att_control.zero();
}

bool
MulticopterAttitudeControl::init()
{
	if (!_vehicle_attitude_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}
	reset_mpc_state();
	return true;
}

void
MulticopterAttitudeControl::parameters_updated()
{
	_attitude_control.setProportionalGain(
		Vector3f(_param_mc_roll_p.get(), _param_mc_pitch_p.get(), _param_mc_yaw_p.get()),
		_param_mc_yaw_weight.get());

	using math::radians;
	_attitude_control.setRateLimit(Vector3f(
		radians(_param_mc_rollrate_max.get()),
		radians(_param_mc_pitchrate_max.get()),
		radians(_param_mc_yawrate_max.get())));

	if (!PX4_ISFINITE(_hover_thrust_estimate)) {
		_hover_thrust_slew_rate.setForcedValue(_param_mpc_thr_hover.get());
	}

	_man_tilt_max = math::radians(_param_mpc_man_tilt_max.get());
}

// =============================================================================
// QP SOLVER — Hildreth's method
// =============================================================================

void MulticopterAttitudeControl::QPhild()
{
	// Step 1 — tempEF = E_inv * F  (6×1)
	for (int i = 0; i < 6; i++) {
		_tempEF(i) = 0.f;
		for (int j = 0; j < 6; j++)
			_tempEF(i) += _E_inv(i,j) * _F(j);
	}

	// Step 2 — Kvec = CC * tempEF + d_local  (24×1)
	for (int i = 0; i < 24; i++) {
		_Kvec(i) = 0.f;
		for (int j = 0; j < 6; j++)
			_Kvec(i) += _CC(i,j) * _tempEF(j);
		_Kvec(i) += _d_local(i);
	}

	for (int i = 0; i < 24; i++) {
		_lambda(i)      = 0.f;
		_lambda_prev(i) = 0.f;
	}

	for (int km = 0; km < 10; km++) {
		for (int i = 0; i < 24; i++) _lambda_prev(i) = _lambda(i);

		for (int i = 0; i < 24; i++) {
			float Tii = _T(i,i);
			if (fabsf(Tii) < 1e-10f) continue;

			float dot = 0.f;
			for (int k = 0; k < 24; k++)
				if (k != i) dot += _T(k,i) * _lambda(k);
			float la = -(dot + _Kvec(i)) / _T(i,i);
			_lambda(i) = (la < 0.f) ? 0.f : la;
		}

		float al = 0.f;
		for (int i = 0; i < 24; i++) {
			float d = _lambda(i) - _lambda_prev(i);
			al += d * d;
		}
		if (al < 0.001f) break;
	}

	// Step 4 — DeltaU = -E_inv*(F + CC'*lambda)
	for (int i = 0; i < 6; i++) {
		_tempVec6(i) = 0.f;
		for (int j = 0; j < 24; j++)
			_tempVec6(i) += _CC(j,i) * _lambda(j);
	}
	for (int i = 0; i < 6; i++) {
		_tempV6_2(i) = 0.f;
		for (int j = 0; j < 6; j++)
			_tempV6_2(i) += _E_inv(i,j) * _tempVec6(j);
	}
	for (int i = 0; i < 6; i++)
		_DeltaU(i) = -_tempEF(i) - _tempV6_2(i);
}

// =============================================================================
// RATE CONTROLLER
// =============================================================================

void MulticopterAttitudeControl::control_attitude_rates(float dt,
        const matrix::Vector3f &rates, const matrix::Quatf &q)
{

	_Ad.zero();
	_Ad(0,0) = 1.f; _Ad(0,1) = dt;
	_Ad(1,1) = 1.f;
	_Ad(2,2) = 1.f; _Ad(2,3) = dt;
	_Ad(3,3) = 1.f;
	_Ad(4,4) = 1.f; _Ad(4,5) = dt;
	_Ad(5,5) = 1.f;

	const matrix::Eulerf euler{q};

	_x(0,0) = euler.phi();    // roll angle
	_x(1,0) = rates(0);       // roll rate  p
	_x(2,0) = euler.theta();  // pitch angle
	_x(3,0) = rates(1);       // pitch rate q
	_x(4,0) = euler.psi();    // yaw angle
	_x(5,0) = rates(2);       // yaw rate   r

	_des(0,0) = _rates_sp(0);
	_des(1,0) = _rates_sp(1);
	_des(2,0) = _rates_sp(2);

	for(int i=0;i<2;i++)
	{
		_temp15 = _Rs * _des;
		_temp15 -= _P * _Xf;
		_temp6  = _H_trans * _temp15;
		for (int j = 0; j < 6; j++) _F(j) = -2.f * _temp6(j,0);

		_d_local = _dd + _dupast * _u;

		QPhild();

		_delta_first(0,0) = _DeltaU(0);
		_delta_first(1,0) = _DeltaU(1);
		_delta_first(2,0) = _DeltaU(2);

		_u += _delta_first;

		// NaN guard before constrain
		if (!PX4_ISFINITE(_u(0,0)) || !PX4_ISFINITE(_u(1,0)) || !PX4_ISFINITE(_u(2,0))) {
			PX4_ERR("MPC: NaN in _u, resetting");
			reset_mpc_state();
			break;
		}

		// Clamp u to hard physical limits before state propagation.
		// This prevents constraint violation from accumulating in _x.
		_u(0,0) = math::constrain(_u(0,0), -0.7794f,  0.7794f);
		_u(1,0) = math::constrain(_u(1,0), -0.7794f,  0.7794f);
		_u(2,0) = math::constrain(_u(2,0), -0.1935f,  0.1935f);

		_x_prev = _x;
		_x = _Ad * _x + _Bd * _u;
		_y_out = _Cd * _x;

		_x_diff = _x - _x_prev;

		for (int k = 0; k < 6; k++) _Xf(k,0) = _x_diff(k,0);
		_Xf(6,0) = _y_out(0,0);
		_Xf(7,0) = _y_out(1,0);
		_Xf(8,0) = _y_out(2,0);
	}
	// -----------------------------------------------------------------
	// Output — saturate and normalise to [-1, 1]
	// -----------------------------------------------------------------

	// The clamping above already ensures _u is within [−umax, +umax].
	_att_control(0) = _u(0,0);
	_att_control(1) = _u(1,0);
	_att_control(2) = _u(2,0);

	const float umax[3] = {0.7794f, 0.7794f, 0.1935f};

	for (int k = 0; k < 3; k++) {
		const float range = 2.f * umax[k];
		// range is always > 0 for our umax values, but guard anyway
		if (range > 1e-6f)
			_att_control(k) = 2.f * ((_u(k,0) + umax[k]) / range) - 1.f;
		else
			_att_control(k) = 0.f;
	}
}

// =============================================================================
// THROTTLE CURVE
// =============================================================================

float
MulticopterAttitudeControl::throttle_curve(float throttle_stick_input)
{
	float thrust = 0.f;

	switch (_param_mpc_thr_curve.get()) {
	case 1:
		thrust = math::interpolate(throttle_stick_input, -1.f, 1.f,
			_manual_throttle_minimum.getState(), _param_mpc_thr_max.get());
		break;
	case 2:
		thrust = math::interpolateNXY(throttle_stick_input,
			{-1.f, 0.f, 1.f},
			{_manual_throttle_minimum.getState(), _param_mpc_thr_hover.get(), _param_mpc_thr_max.get()});
		break;
	default:
		thrust = math::interpolateNXY(throttle_stick_input,
			{-1.f, 0.f, 1.f},
			{_manual_throttle_minimum.getState(), _hover_thrust_slew_rate.getState(), _param_mpc_thr_max.get()});
		break;
	}

	return math::min(thrust, _manual_throttle_maximum.getState());
}

// =============================================================================
// ATTITUDE SETPOINT GENERATOR
// =============================================================================

void
MulticopterAttitudeControl::generate_attitude_setpoint(const Quatf &q, float dt)
{
	vehicle_attitude_setpoint_s attitude_setpoint{};

	const bool arming_gesture = (_manual_control_setpoint.throttle < -.9f) && (_param_mc_airmode.get() != 2);
	if (arming_gesture) { _man_yaw_sp = NAN; }

	const float yaw = Eulerf(q).psi();
	const float yaw_stick_input = math::expo_deadzone(_manual_control_setpoint.yaw, .6f, _param_man_deadzone.get());
	_stick_yaw.generateYawSetpoint(attitude_setpoint.yaw_sp_move_rate, _man_yaw_sp,
	                               yaw_stick_input, yaw, _heading_good_for_control, dt);

	_man_roll_input_filter.setParameters(dt, _param_mc_man_tilt_tau.get());
	_man_pitch_input_filter.setParameters(dt, _param_mc_man_tilt_tau.get());

	Vector2f v = Vector2f(
		 _man_roll_input_filter.update(_manual_control_setpoint.roll  * _man_tilt_max),
		-_man_pitch_input_filter.update(_manual_control_setpoint.pitch * _man_tilt_max));

	float v_norm = v.norm();
	if (v_norm > _man_tilt_max) { v *= _man_tilt_max / v_norm; }

	Quatf q_sp_rp = AxisAnglef(v(0), v(1), 0.f);
	const float yaw_setpoint = PX4_ISFINITE(_man_yaw_sp) ? _man_yaw_sp : yaw;
	const Quatf q_sp_yaw(cosf(yaw_setpoint / 2.f), 0.f, 0.f, sinf(yaw_setpoint / 2.f));

	if (_vtol) {
		AttitudeControlMath::correctTiltSetpointForYawError(q_sp_rp, q, q_sp_yaw);
	}

	Quatf q_sp = q_sp_yaw * q_sp_rp;
	q_sp.copyTo(attitude_setpoint.q_d);

	attitude_setpoint.thrust_body[2] = -throttle_curve(_manual_control_setpoint.throttle);
	attitude_setpoint.timestamp = hrt_absolute_time();
	_vehicle_attitude_setpoint_pub.publish(attitude_setpoint);
}

// =============================================================================
// MAIN RUN LOOP
// =============================================================================

void
MulticopterAttitudeControl::Run()
{
	if (should_exit()) {
		_vehicle_attitude_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);

	if (_parameter_update_sub.updated()) {
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);
		updateParams();
		parameters_updated();
	}

	if (_hover_thrust_estimate_sub.updated()) {
		hover_thrust_estimate_s hover_thrust_estimate;
		if (_hover_thrust_estimate_sub.update(&hover_thrust_estimate)) {
			if (hover_thrust_estimate.valid) {
				_hover_thrust_estimate = math::constrain(hover_thrust_estimate.hover_thrust, .05f, .9f);
			} else {
				_hover_thrust_estimate = _param_mpc_thr_hover.get();
			}
		}
	}

	vehicle_attitude_s v_att;

	if (_vehicle_attitude_sub.update(&v_att)) {

		const float dt = math::constrain(
			((v_att.timestamp_sample - _last_run) * 1e-6f), 0.0002f, 0.02f);
		_last_run = v_att.timestamp_sample;

		const Quatf q{v_att.q};

		_manual_control_setpoint_sub.update(&_manual_control_setpoint);
		_vehicle_control_mode_sub.update(&_vehicle_control_mode);

		if (_vehicle_status_sub.updated()) {
			vehicle_status_s vehicle_status;
			if (_vehicle_status_sub.copy(&vehicle_status)) {
				_vehicle_type_rotary_wing = (vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING);
				_vtol                     = vehicle_status.is_vtol;
				_vtol_in_transition_mode  = vehicle_status.in_transition_mode;
				_vtol_tailsitter          = vehicle_status.is_vtol_tailsitter;
				const bool armed = (vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED);
				const bool just_disarmed = _last_armed_state && !armed;
				_last_armed_state = armed;
				_spooled_up = armed && hrt_elapsed_time(&vehicle_status.armed_time) > _param_com_spoolup_time.get() * 1_s;
				if (just_disarmed) { reset_mpc_state(); }
			}
		}

		if (_vehicle_land_detected_sub.updated()) {
			vehicle_land_detected_s vehicle_land_detected;
			if (_vehicle_land_detected_sub.copy(&vehicle_land_detected)) {
				const bool just_landed = !_landed && vehicle_land_detected.landed;
				_landed = vehicle_land_detected.landed;
				if (just_landed) { reset_mpc_state(); }
			}
		}

		if (_vehicle_local_position_sub.updated()) {
			vehicle_local_position_s vehicle_local_position;
			if (_vehicle_local_position_sub.copy(&vehicle_local_position)) {
				_heading_good_for_control = vehicle_local_position.heading_good_for_control;
			}
		}

		const bool is_hovering            = (_vehicle_type_rotary_wing && !_vtol_in_transition_mode);
		const bool is_tailsitter_transition= (_vtol_tailsitter && _vtol_in_transition_mode);
		const bool run_att_ctrl            = _vehicle_control_mode.flag_control_attitude_enabled
		                                     && (is_hovering || is_tailsitter_transition);

		if (run_att_ctrl) {
			if (_vehicle_control_mode.flag_control_manual_enabled
			    && !_vehicle_control_mode.flag_control_altitude_enabled
			    && !_vehicle_control_mode.flag_control_velocity_enabled
			    && !_vehicle_control_mode.flag_control_position_enabled) {
				generate_attitude_setpoint(q, dt);
			} else {
				_man_roll_input_filter.reset(0.f);
				_man_pitch_input_filter.reset(0.f);
				_man_yaw_sp = Eulerf(q).psi();
			}

			if (_vehicle_attitude_setpoint_sub.updated()) {
				vehicle_attitude_setpoint_s vehicle_attitude_setpoint;
				if (_vehicle_attitude_setpoint_sub.copy(&vehicle_attitude_setpoint)
				    && (vehicle_attitude_setpoint.timestamp > _last_attitude_setpoint)) {
					_attitude_control.setAttitudeSetpoint(
						Quatf(vehicle_attitude_setpoint.q_d),
						vehicle_attitude_setpoint.yaw_sp_move_rate);
					_thrust_setpoint_body  = Vector3f(vehicle_attitude_setpoint.thrust_body);
					_last_attitude_setpoint = vehicle_attitude_setpoint.timestamp;
				}
			}

			// Outer Attitude P-loop → desired angular rates
			_rates_sp = _attitude_control.update(q);

			if (_quat_reset_counter != v_att.quat_reset_counter) {
				const Quatf delta_q_reset(v_att.delta_q_reset);
				const float delta_psi = Eulerf(delta_q_reset).psi();
				if (PX4_ISFINITE(_man_yaw_sp)) {
					_man_yaw_sp = wrap_pi(_man_yaw_sp + delta_psi);
				}
				if (v_att.timestamp > _last_attitude_setpoint) {
					_attitude_control.adaptAttitudeSetpoint(delta_q_reset);
				}
				_quat_reset_counter = v_att.quat_reset_counter;
			}

			// MPC inner-loop rate controller
			vehicle_angular_velocity_s ang_vel{};
			if (_vehicle_angular_velocity_sub.copy(&ang_vel)) {
				matrix::Vector3f rates(ang_vel.xyz[0], ang_vel.xyz[1], ang_vel.xyz[2]);
				control_attitude_rates(dt, rates, q);

				_rates_sp = Vector3f(_att_control(0), _att_control(1), _att_control(2));
			}

			const hrt_abstime now = hrt_absolute_time();
			autotune_attitude_control_status_s pid_autotune;
			if (_autotune_attitude_control_status_sub.copy(&pid_autotune)) {
				if ((pid_autotune.state == autotune_attitude_control_status_s::STATE_ROLL
				     || pid_autotune.state == autotune_attitude_control_status_s::STATE_PITCH
				     || pid_autotune.state == autotune_attitude_control_status_s::STATE_YAW
				     || pid_autotune.state == autotune_attitude_control_status_s::STATE_TEST)
				    && ((now - pid_autotune.timestamp) < 1_s)) {
					_rates_sp += Vector3f(pid_autotune.rate_sp);
				}
			}

			vehicle_rates_setpoint_s rates_setpoint{};
			rates_setpoint.roll  = _rates_sp(0);
			rates_setpoint.pitch = _rates_sp(1);
			rates_setpoint.yaw   = _rates_sp(2);
			_thrust_setpoint_body.copyTo(rates_setpoint.thrust_body);
			rates_setpoint.timestamp = hrt_absolute_time();
			_vehicle_rates_setpoint_pub.publish(rates_setpoint);

		} else {
			_man_roll_input_filter.reset(0.f);
			_man_pitch_input_filter.reset(0.f);
			_man_yaw_sp = Eulerf(q).psi();
		}

		if (_landed) {
			_manual_throttle_minimum.update(0.f, dt);
		} else {
			_manual_throttle_minimum.update(_param_mpc_manthr_min.get(), dt);
		}

		if (_spooled_up) {
			_manual_throttle_maximum.update(1.f, dt);
		} else {
			_manual_throttle_maximum.setForcedValue(0.f);
		}

		if (PX4_ISFINITE(_hover_thrust_estimate)) {
			_hover_thrust_slew_rate.update(_hover_thrust_estimate, dt);
		}
	}

	perf_end(_loop_perf);
}

// =============================================================================
// MODULE BOILERPLATE
// =============================================================================

int MulticopterAttitudeControl::task_spawn(int argc, char *argv[])
{
	bool vtol = false;
	if (argc > 1 && strcmp(argv[1], "vtol") == 0) { vtol = true; }

	MulticopterAttitudeControl *instance = new MulticopterAttitudeControl(vtol);

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;
		if (instance->init()) { return PX4_OK; }
	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;
	return PX4_ERROR;
}

int MulticopterAttitudeControl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int MulticopterAttitudeControl::print_usage(const char *reason)
{
	if (reason) { PX4_WARN("%s\n", reason); }

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Multicopter attitude controller with MPC inner-loop rate controller.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("mc_att_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_ARG("vtol", "VTOL mode", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

extern "C" __EXPORT int mc_att_control_main(int argc, char *argv[])
{
	return MulticopterAttitudeControl::main(argc, argv);
}
