#pragma once

#include <ctre/Phoenix.h>
#include "AutonomousMode.h"

class AutoTest : public AutonomousMode {
  public:
    void begin (TalonSRX &srx_left, TalonSRX &srx_right);
    void run (TalonSRX &srx_left, TalonSRX &srx_right);
    void end (TalonSRX &srx_left, TalonSRX &srx_right);
    void forceEnd (TalonSRX &srx_left, TalonSRX &srx_right);
};