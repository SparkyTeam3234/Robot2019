#pragma once

#include <ctre/Phoenix.h>
#include "AutonomousMode.h"
#include "MathFunction.h"
#include "Motion.h"

class AutonomousModeMotion : public AutonomousMode {
  private:
    Motion motion{0.02};
    int ofs_left=0;
    int ofs_right=0;
    float distance=0;
    float time=0;
    float tolerance=0;
    float k=0;
  public:
    AutonomousModeMotion(float distance,float time,float tolerance,float k) {
      this->distance=distance;
      this->time=time;
      this->tolerance=tolerance;
      this->k=k;
    }
    void begin (TalonSRX &srx_left, TalonSRX &srx_right) {
      ofs_left=srx_left.GetSensorCollection().GetQuadraturePosition();
      ofs_right=srx_right.GetSensorCollection().GetQuadraturePosition();
    }
    void run (TalonSRX &srx_left, TalonSRX &srx_right) {
      int enc_left=srx_left.GetSensorCollection().GetQuadraturePosition()-ofs_left;
      int enc_right=srx_right.GetSensorCollection().GetQuadraturePosition()-ofs_right;
      double power_left=motion.Drive(enc_left,distance,time,tolerance,k);
      double power_right=motion.Drive(enc_right,distance,time,tolerance,k);
      srx_left.Set(ControlMode::PercentOutput, power_left);
      srx_right.Set(ControlMode::PercentOutput, power_right);
      if (motion.GetTime()>time) {
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