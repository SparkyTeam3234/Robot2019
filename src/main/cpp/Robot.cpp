/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"

#include <iostream>

#include <cameraserver/CameraServer.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/Spark.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/SpeedController.h>
#include <frc/SpeedControllerGroup.h>
#include <frc/Joystick.h>
#include <frc/Timer.h>
#include <wpi/raw_ostream.h>
#include <ctre/Phoenix.h>
#include <frc/encoder.h>
#include <frc/SmartDashboard/SmartDashboard.h>
#include <algorithm>
#include "AutonomousMode.h"
#include "AutoTest.h"
#include <cmath>
#include "TPixy.h"



void Robot::RobotInit() {
  driveStickLeft=new frc::Joystick(0);
  driveStickRight=new frc::Joystick(1);
  srx_left.SetInverted(false);
  srx_left.Set(ControlMode::PercentOutput, 0);
  srx_right.Set(ControlMode::PercentOutput, 0);
#if defined(__linux__)
    frc::CameraServer::GetInstance()->StartAutomaticCapture();
#else
    wpi::errs() << "Vision only available on Linux.\n";
    wpi::errs().flush();
#endif
}

/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {}

/**
 * This autonomous (along with the chooser code above) shows how to select
 * between different autonomous modes using the dashboard. The sendable chooser
 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
 * remove all of the chooser code and uncomment the GetString line to get the
 * auto name from the text box below the Gyro.
 *+
 * You can add additional auto modes by adding additional comparisons to the
 * if-else structure below with additional strings. If using the SendableChooser
 * make sure to add them to the chooser code above as well.
 */
void Robot::AutonomousInit() 
{
  srx_left.GetSensorCollection().SetQuadraturePosition(0);
  srx_right.GetSensorCollection().SetQuadraturePosition(0);
}

void Robot::AutonomousPeriodic() 
{
  TeleopPeriodic();
}

void Robot::TeleopInit() 
{
  
}

void Robot::TeleopPeriodic() 
{
  if (autoMode) {
    wpi::errs() << "Truth.\n";
    wpi::errs().flush();
    autoMode->run(srx_left,srx_right);
    if (autoMode->isDone()) {
      autoMode->end(srx_left,srx_right);
      delete autoMode;
      autoMode=0;
    }
    else if (driveStickLeft->GetTrigger()) {
      autoMode->forceEnd(srx_left,srx_right);
      delete autoMode;
      autoMode=0;
    }
  }
  else {
    //m_drive.ArcadeDrive(driveStick->GetY(), driveStick->GetX());
    srx_left.Set(ControlMode::PercentOutput, driveStickLeft->GetY()*std::abs(driveStickLeft->GetY()));
    srx_right.Set(ControlMode::PercentOutput, driveStickRight->GetY()*std::abs(driveStickRight->GetY()));

    if (driveStickRight->GetTrigger()) {
      srx_left.GetSensorCollection().SetQuadraturePosition(0);
      srx_right.GetSensorCollection().SetQuadraturePosition(0);
    }

    if (driveStickLeft->GetRawButton(2)) {
      try {
        autoMode=new AutoTest;
      } catch (std::bad_alloc e) {
        wpi::errs() << "Bad Allocation of Autonomous Mode.\n";
        wpi::errs().flush();
      }
    }
  }
  frc::SmartDashboard::PutNumber("Left",srx_left.GetSensorCollection().GetQuadraturePosition());
  frc::SmartDashboard::PutNumber("Right",srx_right.GetSensorCollection().GetQuadraturePosition());
}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
