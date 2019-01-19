#pragma once

#include <ctre/Phoenix.h>

class AutonomousMode {
  protected:
    bool done=false;
  public:
    virtual void begin (TalonSRX &srx_left, TalonSRX &srx_right) =0;
    virtual void run (TalonSRX &srx_left, TalonSRX &srx_right) =0;
    virtual void end (TalonSRX &srx_left, TalonSRX &srx_right) =0;
    virtual void forceEnd (TalonSRX &srx_left, TalonSRX &srx_right) =0;
    bool isDone();
};