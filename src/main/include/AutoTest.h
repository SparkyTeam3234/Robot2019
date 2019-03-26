#pragma once

#include <ctre/Phoenix.h>
#include "AutonomousMode.h"

class AutoTest : public AutonomousMode {
  public:
    void begin (TalonSRX &srx_left, TalonSRX &srx_right) {
      srx_left.GetSensorCollection().SetQuadraturePosition(0);
      srx_right.GetSensorCollection().SetQuadraturePosition(0);
    }
    void run (TalonSRX &srx_left, TalonSRX &srx_right) {
      srx_left.Set(ControlMode::PercentOutput, -0.4);
      srx_right.Set(ControlMode::PercentOutput, -0.4);
      if (srx_left.GetSensorCollection().GetQuadraturePosition()>=16000 || srx_right.GetSensorCollection().GetQuadraturePosition()>=16000) {
        done=true;
      }
    }
    void end (TalonSRX &srx_left, TalonSRX &srx_right) {
      ;
    }
    void forceEnd (TalonSRX &srx_left, TalonSRX &srx_right) {
      ;
    }
};