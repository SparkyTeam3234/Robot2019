#pragma once

#include <ctre/Phoenix.h>
#include "AutonomousMode.h"
#include "Absolute.h"

class AutonomousModeNewMotion: public AutonomousMode {
public:
	int target_left, target_right, relative, tolerance, offset_left,
			offset_right;
	AutonomousModeNewMotion(int target_left, int target_right,
			int tolerance, bool relative) {
		this->target_left = target_left;
		this->target_right = target_right;
		this->tolerance = tolerance;
		this->relative = relative;
	}
	void begin(TalonSRX &srx_left, TalonSRX &srx_right) {
		offset_left = srx_left.GetSensorCollection().GetQuadraturePosition();
		offset_right = srx_left.GetSensorCollection().GetQuadraturePosition();
	}
	void run(TalonSRX &srx_left, TalonSRX &srx_right) {
		srx_left.Set(ControlMode::MotionMagic, target_left);
		srx_right.Set(ControlMode::MotionMagic, target_right);
		if (relative
				&& cabs(
						srx_left.GetSensorCollection().GetQuadraturePosition()
								- target_left - offset_left) < tolerance
				&& cabs(
						srx_right.GetSensorCollection().GetQuadraturePosition()
								- target_right - offset_right) < tolerance) {
			done = true;
		} else if (!relative
				&& cabs(
						srx_left.GetSensorCollection().GetQuadraturePosition()
								- target_left) < tolerance
				&& cabs(
						srx_right.GetSensorCollection().GetQuadraturePosition()
								- target_right) < tolerance) {
			done = true;
		}
	}
	void end(TalonSRX &srx_left, TalonSRX &srx_right) {
		;
	}
	void forceEnd(TalonSRX &srx_left, TalonSRX &srx_right) {
		;
	}
};
