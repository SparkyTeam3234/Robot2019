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
#include <frc/DigitalInput.h>
#include <wpi/raw_ostream.h>
#include <ctre/Phoenix.h>
#include "TPixy2.h"
#include "Pixy2I2C.h"
#include "AutonomousMode.h"
#include <thread>
#include <frc/AnalogInput.h>
#include "rev/CANSparkMax.h"
#include "Motion.h"

class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void TestPeriodic() override;
  void ArcadeDrive();
  void EncoderSave();
  void EncoderLoad();
  //Joysticks
  frc::Joystick *driveStickLeft;
  frc::Joystick *driveStickRight;
  frc::Joystick *otherStick;
  frc::Joystick *auxStick;
  //
  frc::AnalogInput *ultra;
  Motion m_leftDriveMotion{0.02};
  Motion m_rightDriveMotion{0.02};
  Motion m_actuatorMotion{0.2};
  Motion m_liftMotion{0.2};
  //Wheel Left
  VictorSPX srx_wheel_left{1};
  //Drive Left Motor Controllers
  TalonSRX srx_left{2};
  TalonSRX srx_left2{3};
  //Motor 2
  rev::CANSparkMax m_tilt{7, rev::CANSparkMax::MotorType::kBrushed};
  rev::CANSparkMax m_lift{5, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_lift2{6, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_actuator{11, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_carriage{4, rev::CANSparkMax::MotorType::kBrushless};
  //Drive Right Motor Controllers
  TalonSRX srx_right{8};
  TalonSRX srx_right2{9};
  //Wheel Right
  VictorSPX srx_wheel_right{10};

  frc::DigitalInput m_limit{0};

  AutonomousMode *autoMode=0;
  
  rev::CANEncoder m_encoder_tilt = m_tilt.GetEncoder();
  rev::CANEncoder m_encoder_lift = m_lift.GetEncoder();
  rev::CANEncoder m_encoder_lift2 = m_lift2.GetEncoder();
  rev::CANEncoder m_encoder_actuator = m_actuator.GetEncoder();
  rev::CANEncoder m_encoder_carriage = m_carriage.GetEncoder();
  
  TPixy2<Link2I2C> pixy;
  frc::AnalogInput *rangeright;
  frc::AnalogInput *rangeleft;
 private:
  bool tryLines;
  bool SandstormTele=true;
  int autonum=0;
};
