/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
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
 * @file baro_height_control.cpp
 * Control functions for ekf barometric height fusion
 */

#include "ekf.h"

void Ekf::controlBaroHeightFusion(const imuSample &imu_sample)
{
	_fc.baro.available = (_params.ekf2_baro_ctrl != 0);

	static constexpr const char *HGT_SRC_NAME = "baro";

	auto &aid_src = _aid_src_baro_hgt;
	HeightBiasEstimator &bias_est = _baro_b_est;

	bias_est.predict(_dt_ekf_avg);

	baroSample baro_sample;

	if (_baro_buffer && _baro_buffer->pop_first_older_than(imu_sample.time_us, &baro_sample)) {

#if defined(CONFIG_EKF2_BARO_COMPENSATION)
		const float measurement = compensateBaroForDynamicPressure(imu_sample, baro_sample.hgt);
#else
		const float measurement = baro_sample.hgt;
#endif

		const float measurement_var = sq(_params.ekf2_baro_noise);

		const bool measurement_valid = PX4_ISFINITE(measurement) && PX4_ISFINITE(measurement_var);

		if (measurement_valid) {
			if ((_baro_counter == 0) || baro_sample.reset) {
				_baro_lpf.reset(measurement);
				_baro_counter = 1;
				_control_status.flags.baro_fault = false;

			} else {
				_baro_lpf.update(measurement);
				_baro_counter++;
			}

			// Keep re-initialising the pressure offset until the baro has settled. The
			// offset latched here is what ties the height frame to the baro, so a
			// startup step or fast drift latched here stays in the frame for good.
			const bool baro_settled = updateBaroSettled(baro_sample, measurement);

			if ((_baro_counter <= _obs_buffer_length) || !baro_settled) {
				// Initialize the pressure offset (included in the baro bias)
				bias_est.setBias(-_gpos.altitude() + _baro_lpf.getState());
			}
		}

		// vertical position innovation - baro measurement has opposite sign to earth z axis
		updateVerticalPositionAidStatus(aid_src,
						baro_sample.time_us,
						-(measurement - bias_est.getBias()),      // observation
						measurement_var + bias_est.getBiasVar(),  // observation variance
						math::max(_params.ekf2_baro_gate, 1.f)); // innovation gate

		// Compensate for positive static pressure transients (negative vertical position innovations)
		// caused by rotor wash ground interaction by applying a temporary deadzone to baro innovations.
		if (_control_status.flags.gnd_effect && (_params.ekf2_gnd_eff_dz > 0.f)) {

			const float deadzone_start = 0.0f;
			const float deadzone_end = deadzone_start + _params.ekf2_gnd_eff_dz;

			if (aid_src.innovation < -deadzone_start) {
				if (aid_src.innovation <= -deadzone_end) {
					aid_src.innovation += deadzone_end;

				} else {
					aid_src.innovation = -deadzone_start;
				}
			}
		}

		// update the bias estimator before updating the main filter but after
		// using its current state to compute the vertical position innovation
		if (measurement_valid) {
			bias_est.setMaxStateNoise(sqrtf(measurement_var));
			bias_est.setProcessNoiseSpectralDensity(_params.baro_bias_nsd);
			bias_est.fuseBias(measurement - _gpos.altitude(), measurement_var + P(State::pos.idx + 2, State::pos.idx + 2));
		}

		// determine if we should use height aiding
		const bool continuing_conditions_passing = _fc.baro.intended()
				&& measurement_valid
				&& (_baro_counter > _obs_buffer_length)
				&& _baro_settled
				&& !_control_status.flags.baro_fault;

		const bool starting_conditions_passing = continuing_conditions_passing
				&& isNewestSampleRecent(_time_last_baro_buffer_push, 2 * BARO_MAX_INTERVAL);

		if (_control_status.flags.baro_hgt) {

			if (continuing_conditions_passing) {

				fuseVerticalPosition(aid_src);

				const bool is_fusion_failing = isTimedOut(aid_src.time_last_fuse, _params.hgt_fusion_timeout_max);

				if (isHeightResetRequired() && (_height_sensor_ref == HeightSensor::BARO)) {
					// All height sources are failing
					ECL_WARN("%s height fusion reset required, all height sources failing", HGT_SRC_NAME);

					_information_events.flags.reset_hgt_to_baro = true;
					resetAltitudeTo(_baro_lpf.getState() - bias_est.getBias(), measurement_var);
					bias_est.setBias(-_gpos.altitude() + _baro_lpf.getState());
					resetAidSourceStatusZeroInnovation(aid_src);

					// reset vertical velocity if no valid sources available
					if (!isVerticalVelocityAidingActive()) {
						resetVerticalVelocityToZero();
					}

					aid_src.time_last_fuse = imu_sample.time_us;

				} else if (is_fusion_failing) {
					ECL_WARN("stopping %s height fusion, fusion failing", HGT_SRC_NAME);
					stopBaroHgtFusion();

					if (isRecent(_time_last_hgt_fuse, _params.hgt_fusion_timeout_max)) {
						// Some other height source is still working
						_control_status.flags.baro_fault = true;
					}
				}

			} else {
				ECL_WARN("stopping %s height fusion, continuing conditions failing", HGT_SRC_NAME);
				stopBaroHgtFusion();
			}

		} else {
			if (starting_conditions_passing) {
				if (_params.ekf2_hgt_ref == static_cast<int32_t>(HeightSensor::BARO)) {
					ECL_INFO("starting %s height fusion, resetting height", HGT_SRC_NAME);
					_height_sensor_ref = HeightSensor::BARO;

					_information_events.flags.reset_hgt_to_baro = true;
					initialiseAltitudeTo(measurement, measurement_var);
					bias_est.reset();
					resetAidSourceStatusZeroInnovation(aid_src);

				} else {
					ECL_INFO("starting %s height fusion", HGT_SRC_NAME);
					bias_est.setBias(-_gpos.altitude() + _baro_lpf.getState());
				}

				aid_src.time_last_fuse = imu_sample.time_us;
				bias_est.setFusionActive();
				_control_status.flags.baro_hgt = true;
			}
		}

	} else if (_control_status.flags.baro_hgt
		   && !isNewestSampleRecent(_time_last_baro_buffer_push, 2 * BARO_MAX_INTERVAL)) {
		// No data anymore. Stop until it comes back.
		ECL_WARN("stopping %s height fusion, no data", HGT_SRC_NAME);
		stopBaroHgtFusion();
	}
}

bool Ekf::updateBaroSettled(const baroSample &baro_sample, const float measurement)
{
	// Latched: once the baro has been used, later noise (prop wash, vibration) must
	// never take it away again. Only Ekf::reset() clears this.
	if (_baro_settled) {
		return true;
	}

	if (_params.ekf2_baro_stb_sd <= 0.f) {
		// check disabled: stock behaviour
		_baro_settled = true;
		return true;
	}

	const uint64_t now = baro_sample.time_us;

	if ((_baro_settle_first_us == 0) || baro_sample.reset || (now <= _baro_settle_last_us)) {
		// first sample, a baro switch or a time jump: start the check again
		_baro_settle_first_us = now;
		_baro_settle_last_us = now;
		_baro_settle_quiet_us = now;
		_baro_settle_mean = measurement;
		_baro_settle_var = 0.f;
		return false;
	}

	// Exponentially weighted mean and variance with a 1 s time constant. The spread
	// sqrt(var) responds to both failure shapes seen on this board:
	//  - a step of S metres jumps it to ~0.24 S (16 Hz baro), then it decays over ~6 s
	//  - a ramp of r m/s holds it at ~r * 1 s, because the mean lags the ramp
	static constexpr float BARO_SETTLE_TAU = 1.f; // (s)
	const float dt = 1e-6f * (float)(now - _baro_settle_last_us);
	_baro_settle_last_us = now;

	const float alpha = dt / (BARO_SETTLE_TAU + dt);
	const float dev = measurement - _baro_settle_mean;
	_baro_settle_mean += alpha * dev;
	_baro_settle_var = (1.f - alpha) * (_baro_settle_var + alpha * sq(dev));

	const float spread = sqrtf(_baro_settle_var);

	if (spread >= _params.ekf2_baro_stb_sd) {
		// restart the hold time
		_baro_settle_quiet_us = 0;

	} else if (_baro_settle_quiet_us == 0) {
		_baro_settle_quiet_us = now;
	}

	const uint64_t hold_us = (uint64_t)(1e6f * _params.ekf2_baro_stb_t);
	const uint64_t timeout_us = (uint64_t)(1e6f * _params.ekf2_baro_stb_to);
	const float waited_s = 1e-6f * (float)(now - _baro_settle_first_us);

	if ((_baro_settle_quiet_us != 0) && (now - _baro_settle_quiet_us >= hold_us)) {
		ECL_INFO("baro settled after %.1f s (spread %.2f m)", (double)waited_s, (double)spread);
		_baro_settled = true;

	} else if (now - _baro_settle_first_us >= timeout_us) {
		ECL_WARN("baro not settled after %.0f s (spread %.2f m), using it anyway", (double)waited_s, (double)spread);
		_baro_settled = true;
	}

	return _baro_settled;
}

void Ekf::resetBaroSettled()
{
	_baro_settled = false;
	_baro_settle_mean = 0.f;
	_baro_settle_var = 0.f;
	_baro_settle_first_us = 0;
	_baro_settle_last_us = 0;
	_baro_settle_quiet_us = 0;
}

void Ekf::stopBaroHgtFusion()
{
	if (_control_status.flags.baro_hgt) {

		if (_height_sensor_ref == HeightSensor::BARO) {
			_height_sensor_ref = HeightSensor::UNKNOWN;
		}

		_baro_b_est.setFusionInactive();

		_control_status.flags.baro_hgt = false;
	}
}

#if defined(CONFIG_EKF2_BARO_COMPENSATION)
float Ekf::compensateBaroForDynamicPressure(const imuSample &imu_sample, const float baro_alt_uncompensated) const
{
	if (_control_status.flags.wind && isLocalHorizontalPositionValid()) {
		// calculate static pressure error = Pmeas - Ptruth
		// model position error sensitivity as a body fixed ellipse with a different scale in the positive and
		// negative X and Y directions. Used to correct baro data for positional errors

		// Calculate airspeed in body frame
		const Vector3f angular_velocity = (imu_sample.delta_ang / imu_sample.delta_ang_dt) - _state.gyro_bias;
		const Vector3f vel_imu_rel_body_ned = _R_to_earth * (angular_velocity % _params.imu_pos_body);
		const Vector3f velocity_earth = _state.vel - vel_imu_rel_body_ned;

		const Vector3f wind_velocity_earth(_state.wind_vel(0), _state.wind_vel(1), 0.0f);

		const Vector3f airspeed_earth = velocity_earth - wind_velocity_earth;

		const Vector3f airspeed_body = _state.quat_nominal.rotateVectorInverse(airspeed_earth);

		const Vector3f K_pstatic_coef(
			airspeed_body(0) >= 0.f ? _params.ekf2_pcoef_xp : _params.ekf2_pcoef_xn,
			airspeed_body(1) >= 0.f ? _params.ekf2_pcoef_yp : _params.ekf2_pcoef_yn,
			_params.ekf2_pcoef_z);

		const Vector3f airspeed_squared = matrix::min(airspeed_body.emult(airspeed_body), sq(_params.ekf2_aspd_max));

		const float pstatic_err = 0.5f * _air_density * (airspeed_squared.dot(K_pstatic_coef));

		// correct baro measurement using pressure error estimate and assuming sea level gravity
		return baro_alt_uncompensated + pstatic_err / (_air_density * CONSTANTS_ONE_G);
	}

	// otherwise return the uncorrected baro measurement
	return baro_alt_uncompensated;
}
#endif // CONFIG_EKF2_BARO_COMPENSATION
