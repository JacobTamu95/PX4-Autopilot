/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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

#include "ActuatorGroupPreflightCheck.hpp"

bool ActuatorGroupPreflightCheck::isThrust(uint8_t group)
{
	// Thrust groups require different handling for a couple reasons:
	//  - requires being armed to be useful
	//  - want longer duration (thrust can have larger slew rate)
	return group == vehicle_command_s::ACTUATOR_TEST_GROUP_X_THRUST
	       || group == vehicle_command_s::ACTUATOR_TEST_GROUP_Y_THRUST
	       || group == vehicle_command_s::ACTUATOR_TEST_GROUP_Z_THRUST;
}

bool ActuatorGroupPreflightCheck::isKnownGroup(uint8_t group)
{
	switch (group) {
	case vehicle_command_s::ACTUATOR_TEST_GROUP_ROLL_TORQUE:
	case vehicle_command_s::ACTUATOR_TEST_GROUP_PITCH_TORQUE:
	case vehicle_command_s::ACTUATOR_TEST_GROUP_YAW_TORQUE:
	case vehicle_command_s::ACTUATOR_TEST_GROUP_COLLECTIVE_TILT:
	case vehicle_command_s::ACTUATOR_TEST_GROUP_X_THRUST:
	case vehicle_command_s::ACTUATOR_TEST_GROUP_Y_THRUST:
	case vehicle_command_s::ACTUATOR_TEST_GROUP_Z_THRUST:
		return true;

	default:
		return false;
	}
}

const char *ActuatorGroupPreflightCheck::validateCommand(uint8_t group, bool is_tiltrotor, uint8_t &ack_result)
{
	// Permanent failures: the airframe does not support the command
	ack_result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_UNSUPPORTED;

	if (!isKnownGroup(group)) {
		return "unknown actuator group";
	}

	if (group == vehicle_command_s::ACTUATOR_TEST_GROUP_COLLECTIVE_TILT && !is_tiltrotor) {
		ack_result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_UNSUPPORTED;
		return "not a tiltrotor";
	}

	// Genuine internal failure: we could not read the state we need
	ack_result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_FAILED;

	// Transient: arming state mismatch. The user can change it and retry.
	ack_result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED;

	actuator_armed_s actuator_armed{};
	_armed_sub.copy(&actuator_armed);

	if (isThrust(group)) {

		if (!actuator_armed.armed) {
			return "thrust test requires armed";
		}

	} else {

		if (actuator_armed.armed) {
			return "armed";
		}

		if (!actuator_armed.prearmed) {
			return "not prearmed";
		}
	}

	// Transient: landed state mismatch
	vehicle_land_detected_s vehicle_land_detected{};
	_vehicle_land_detected_sub.copy(&vehicle_land_detected);

	if (!vehicle_land_detected.landed) {
		return "not landed";
	}

	return nullptr;
}

void ActuatorGroupPreflightCheck::handleCommand(hrt_abstime now, bool is_tiltrotor)
{
	vehicle_command_s vehicle_command;

	if (!_vehicle_command_sub.update(&vehicle_command)) { return; }

	if (vehicle_command.command != vehicle_command_s::VEHICLE_CMD_ACTUATOR_GROUP_TEST) { return; }

	const uint8_t group = (uint8_t) lroundf(vehicle_command.param1);

	uint8_t reject_result;  // Is written to by validateCommand
	const char *reject_reason = validateCommand(group, is_tiltrotor, reject_result);

	if (reject_reason) {
		PX4_WARN("Actuator group preflight check rejected (%s)", reject_reason);
		// Ack the rejected command directly without changing _last_command, so any check
		// currently running can still respond to its own requester.
		sendAck(vehicle_command, reject_result, now);
		return;
	}

	if (_running) {
		// Cancel the previous check, still targeted at its original requester.
		sendAck(_last_command, vehicle_command_ack_s::VEHICLE_CMD_RESULT_CANCELLED, now);
	}

	// Torque and thrust have inputs in [-1, 1]. Tilt expects [0, 1] (0=vertical, 1=horizontal);
	const float min_input = group == vehicle_command_s::ACTUATOR_TEST_GROUP_COLLECTIVE_TILT ? 0.f : -1.f;

	_group = group;
	_requires_armed = isThrust(group);
	_input = math::constrain(vehicle_command.param2, min_input, 1.f);
	_started = now;
	_running = true;

	_last_command = vehicle_command;
	sendAck(_last_command, vehicle_command_ack_s::VEHICLE_CMD_RESULT_IN_PROGRESS, now);
}

void ActuatorGroupPreflightCheck::updateState(hrt_abstime now)
{
	if (!_running) { return; }

	actuator_armed_s actuator_armed{};
	vehicle_land_detected_s vehicle_land_detected{};

	if (_armed_sub.copy(&actuator_armed) && _vehicle_land_detected_sub.copy(&vehicle_land_detected)) {

		// Cancel if any of the conditions we depended on at start are lost
		// (thrust -> armed, torque/tilt -> pre-armed, always !landed).
		const bool armed_lost = actuator_armed.armed != _requires_armed;
		const bool prearmed_lost = !_requires_armed && !actuator_armed.prearmed;

		if (armed_lost || prearmed_lost || !vehicle_land_detected.landed) {
			_running = false;
			sendAck(_last_command, vehicle_command_ack_s::VEHICLE_CMD_RESULT_CANCELLED, now);
			return;
		}
	}

	if (now - _started >= PREFLIGHT_CHECK_DURATION) {
		_running = false;
		sendAck(_last_command, vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED, now);
	}
}

void ActuatorGroupPreflightCheck::applyOverrides(matrix::Vector<float, NUM_AXES> (&c)[MAX_NUM_MATRICES],
		bool is_vtol, ActuatorEffectiveness &effectiveness)
{
	bool do_override_tilt = false;
	int override_index = -1;

	// Map vehicle command constants to indices of the c matrix used in ControlAllocator
	switch (_group) {
	case vehicle_command_s::ACTUATOR_TEST_GROUP_ROLL_TORQUE:
		override_index = 0; break;

	case vehicle_command_s::ACTUATOR_TEST_GROUP_PITCH_TORQUE:
		override_index = 1; break;

	case vehicle_command_s::ACTUATOR_TEST_GROUP_YAW_TORQUE:
		override_index = 2; break;

	case vehicle_command_s::ACTUATOR_TEST_GROUP_X_THRUST:
		override_index = 3; break;

	case vehicle_command_s::ACTUATOR_TEST_GROUP_Y_THRUST:
		override_index = 4; break;

	case vehicle_command_s::ACTUATOR_TEST_GROUP_Z_THRUST:
		override_index = 5; break;

	case vehicle_command_s::ACTUATOR_TEST_GROUP_COLLECTIVE_TILT:
		do_override_tilt = _running;
		break;
	}

	// Always update tilt override, even when not running. We need an
	// explicit call with do_override_tilt = false to undo the override.
	effectiveness.overrideCollectiveTilt(do_override_tilt, _input);

	if (_running && override_index >= 0) {
		// On a VTOL, control surfaces are in instance 1; thrust always lives in instance 0.
		const int instance = (is_vtol && !isThrust(_group)) ? 1 : 0;
		c[instance](override_index) = _input;
	}
}

void ActuatorGroupPreflightCheck::sendAck(const vehicle_command_s &cmd, uint8_t result, hrt_abstime now)
{
	if (cmd.from_external) {
		vehicle_command_ack_s command_ack{};
		command_ack.timestamp = now;
		command_ack.command = cmd.command;
		command_ack.result = result;
		command_ack.target_system = cmd.source_system;
		command_ack.target_component = cmd.source_component;
		_command_ack_pub.publish(command_ack);
	}
}
