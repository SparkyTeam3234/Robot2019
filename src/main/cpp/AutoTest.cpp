#include <ctre/Phoenix.h>
#include "AutonomousMode.h"
#include "AutoTest.h"

void AutoTest::begin (TalonSRX &srx_left, TalonSRX &srx_right) {
  srx_left.GetSensorCollection().SetQuadraturePosition(0);
  srx_right.GetSensorCollection().SetQuadraturePosition(0);
}

void AutoTest::run (TalonSRX &srx_left, TalonSRX &srx_right) {
  srx_left.Set(ControlMode::PercentOutput, -0.4);
  srx_right.Set(ControlMode::PercentOutput, -0.4);
  if (srx_left.GetSensorCollection().GetQuadraturePosition()>=16000 || srx_right.GetSensorCollection().GetQuadraturePosition()>=16000) {
    done=true;
  }
}

void AutoTest::end (TalonSRX &srx_left, TalonSRX &srx_right) {
  
}

void AutoTest::forceEnd (TalonSRX &srx_left, TalonSRX &srx_right) {
  ;
}