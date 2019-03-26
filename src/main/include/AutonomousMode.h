#pragma once

#include <ctre/Phoenix.h>
#include <vector>
#include <cstdarg>

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

class SequentialAutonomousMode : public AutonomousMode {
  private:
    bool delend;
    std::vector<AutonomousMode*> actions;
    void deleteAll() {
      for (int i=0; i<actions.size(); i++) {
        delete actions.at(i);
      }
    }
    int index=0;
  public:
    SequentialAutonomousMode(bool delend) {
      this->delend=delend;
    }
    void add(AutonomousMode* action) {
      actions.push_back(action);
    }
    void begin (TalonSRX &srx_left, TalonSRX &srx_right) {
      actions.at(index)->begin(srx_left,srx_right);
    }
    void run (TalonSRX &srx_left, TalonSRX &srx_right) {
      actions.at(index)->run(srx_left,srx_right);
      if (actions.at(index)->isDone()) {
        actions.at(index)->end(srx_left,srx_right);
        ++index;
        if (index>=actions.size()) {
          done=true;
        }
        else {
          actions.at(index)->begin(srx_left,srx_right);
        }
      }
    }
    void end (TalonSRX &srx_left, TalonSRX &srx_right) {
      if (delend) {
        deleteAll();
      }
    }
    void forceEnd (TalonSRX &srx_left, TalonSRX &srx_right) {
      actions.at(index)->forceEnd(srx_left,srx_right);
      if (delend) {
        deleteAll();
      }
    }
};
