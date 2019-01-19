/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <string>

#include <cameraserver/CameraServer.h>
#include <frc/TimedRobot.h>
#include <frc/Spark.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/SpeedController.h>
#include <frc/SpeedControllerGroup.h>
#include <frc/Joystick.h>
#include <frc/Timer.h>
#include <frc/encoder.h>
#include <wpi/raw_ostream.h>
#include <ctre/Phoenix.h>
//#include "TPixy.h"
//#include "Pixy2I2C.h"
#include "AutonomousMode.h"

class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TestPeriodic() override;
  void ArcadeDrive();
  //Joysticks
  frc::Joystick *driveStickLeft;
  frc::Joystick *driveStickRight;
  //Drive Motor Controllers
  TalonSRX srx_left{0};
  TalonSRX srx_right{1};
  
  AutonomousMode *autoMode=0;
  //frc::Spark m_left_front{2};
  //frc::Spark m_left_back{3};
  //frc::Spark m_right_front{1};
  //frc::Spark m_right_back{0};
  
  //Drive Motor Controller Groups
  //frc::SpeedControllerGroup m_left{m_left_front,m_left_back};
  //frc::SpeedControllerGroup m_right{m_right_front,m_right_back};
  
  //frc::DifferentialDrive m_drive{m_left,m_right};
  //TPixy<Pixy2I2C> pixy;
  
  //Encoders
  //frc::Encoders left_enc{0,1};
  //frc::Encoders right_enc{2,3};
 private:
  
};
