/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"

#include <iostream>
#include <math.h>

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
#include <frc/DigitalInput.h>
#include <frc/SmartDashboard/SmartDashboard.h>
#include <algorithm>
#include "AutonomousMode.h"
#include "AutonomousModeNewMotion.h"
#include "AutoTest.h"
#include "TPixy2.h"
#include <thread>
#include <frc/AnalogInput.h>
#include "rev/CANSparkMax.h"
#include <cmath>
#include <fstream>
#include "NewMotion.h"
#include "Absolute.h"
#include "Motion.h"

using namespace std;

void Robot::RobotInit() {
	m_lift2.Follow(m_lift, true);

	driveStickLeft = new frc::Joystick(0);
	driveStickRight = new frc::Joystick(1);
	otherStick = new frc::Joystick(2);
	auxStick = new frc::Joystick(3);
	rangeright = new frc::AnalogInput(0);
	rangeleft = new frc::AnalogInput(1);
	//
	srx_left.SetInverted(true);
	srx_left2.SetInverted(true);
	//srx_left.SetSensorPhase(true);
	m_lift2.SetInverted(true);

	srx_left.Set(ControlMode::PercentOutput, 0);
	srx_right.Set(ControlMode::PercentOutput, 0);
	m_encoder_tilt.SetPosition(0.0);
	m_encoder_carriage.SetPosition(0.0);
	m_encoder_lift.SetPosition(0.0);
	m_encoder_lift2.SetPosition(0.0);
	m_encoder_actuator.SetPosition(0.0);
	initializeTalon(&srx_left);
	initializeTalon(&srx_right);
	//srx_left2.Follow(srx_left);
	//srx_right2.Follow(srx_right);
#if defined(__linux__)
	frc::CameraServer::GetInstance()->StartAutomaticCapture();
#else
	wpi::errs() << "Vision only available on Linux.\n";
	wpi::errs().flush();
#endif
	frc::SmartDashboard::PutBoolean("DB/LED 0", false);
	frc::SmartDashboard::PutBoolean("DB/LED 1", false);
	wpi::errs() << "Pixy Init\n";

	pixy.init();
	pixy.setLamp(0, -0);
	//pixy.setServos(500, 750);

	//pixythread = std::thread(DoPixy,pixy);
	//pixythread.detach();
}

/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {
}

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
void Robot::AutonomousInit() {
	srx_left.SetSelectedSensorPosition(0);
	srx_right.SetSelectedSensorPosition(0);

}

void Robot::AutonomousPeriodic() {
	//if(SandstormTele)
	TeleopPeriodic();
	/*
	 else
	 {
	 if(driveStickLeft->GetRawButton(1)||driveStickRight->GetRawButton)
	 SandstormTele=true;
	 switch(autonum);
	 {
	 case 0:
	 {
	 srx_left.Set(ControlMode::PercentOutput, m_leftDriveMotion.Drive(srx_left.GetSelectedSensorPosition(),68224,5,50,0.01);
	 srx_right.Set(ControlMode::PercentOutput, m_rightDriveMotion.Drive(srx_right.GetSelectedSensorPosition(),68224,5,50,0.01);
	 autonum++;
	 break;
	 }
	 case 1:
	 {
	 srx_left.Set(ControlMode::PercentOutput, m_leftDriveMotion.Drive(srx_left.GetSelectedSensorPosition(),68224,5,50,0.01);
	 srx_right.Set(ControlMode::PercentOutput,m_rightDriveMotion.Drive(srx_right.GetSelectedSensorPosition(),68224,5,50,0.01);
	 if(m_leftDriveMotion.GetDone()||m_rightDriveMotion.GetDone()) autonum++;
	 break;
	 }
	 case 2:
	 {
	 srx_left.Set(ControlMode::PercentOutput, 0);
	 srx_right.Set(ControlMode::PercentOutput,0);
	 }
	 }
	 }
	 */
}

void Robot::TeleopInit() {
	m_actuator.GetEncoder().SetPosition(0);
}

void Robot::TeleopPeriodic() {

	if (autoMode) {
		autoMode->run(srx_left, srx_right);
		if (autoMode->isDone()) {
			autoMode->end(srx_left, srx_right);
			delete autoMode;
			autoMode = 0;
		} else if (driveStickLeft->GetTrigger()
				|| driveStickRight->GetTrigger()) {
			autoMode->forceEnd(srx_left, srx_right);
			delete autoMode;
			autoMode = 0;
		}
	} else {
		//m_drive.ArcadeDrive(driveStick->GetY(), driveStick->GetX());

		if (otherStick->GetRawButton(2)) {
			//carriage - south is back/contract, north is out/extend;
			double sy = otherStick->GetY();
			if (m_encoder_carriage.GetPosition() <= -80) {
				sy = (sy < 0.0) ? 0.0 : sy;
			}
			if (m_encoder_carriage.GetPosition() >= 0) {
				sy = (sy > 0.0) ? 0.0 : sy;
			}
			//if (m_limit.Get()) {
			m_carriage.Set(0.3 * sy * cabs(sy));
			/*} else {
			 m_carriage.Set(0.3*sy * cabs(sy));
			 }*/
		} else if (otherStick->GetRawButton(11)) {
			double sy = otherStick->GetY();
			m_carriage.Set(0.3 * sy * cabs(sy));

		} else {
			m_carriage.Set(0.0);
		}
		if (otherStick->GetRawButton(4)) {
			//claw - south is open, north is close

			const double clawOpenEncLimit = -76.122;
			const double clawCloseEncLimit = 0.0;

			double encPosition = m_actuator.GetEncoder().GetPosition();
			frc::SmartDashboard::PutNumber("Actuator Position", encPosition);
			//if we're opening and the encoder is not past the lower limit
			//or we're closing and the encoder is not past the upper limit
			//bool openClawAllowed = (otherStick->GetY() > 0) && (encPosition > clawOpenEncLimit);
			//bool closeClawAllowed = (otherStick->GetY() < 0) && (encPosition < clawCloseEncLimit);

			//if (openClawAllowed || closeClawAllowed)
			double sy = otherStick->GetY();
			if (m_encoder_carriage.GetPosition() <= -80) {
				sy = (sy < 0.0) ? 0.0 : sy;
			}
			if (m_encoder_carriage.GetPosition() >= 0) {
				sy = (sy > 0.0) ? 0.0 : sy;
			}

			{
				m_actuator.Set(-(sy * cabs(sy)));
				//wpi::errs() << "\nActuator Motor Current: "
				//		<< m_actuator.GetOutputCurrent();
				//frc::SmartDashboard::PutNumber("Actuator Amps",
				//		m_actuator.GetOutputCurrent());
			}
		} else if (otherStick->GetRawButton(12)) {
			double sy = otherStick->GetY();
			m_actuator.Set(sy * cabs(sy));
		} else if (auxStick->GetRawButton(3)) {
			m_actuator.Set(
					m_actuatorMotion.FB(-80, m_encoder_actuator.GetPosition()));
		} else if (auxStick->GetRawButton(4)) {
			m_actuator.Set(
					m_actuatorMotion.FB(0, m_encoder_actuator.GetPosition()));
		} else if (auxStick->GetRawButton(5)) {
			m_actuator.Set(
					m_actuatorMotion.FB(-50, m_encoder_actuator.GetPosition()));
		} else if (auxStick->GetRawButton(6)) {
			m_actuator.Set(
					m_actuatorMotion.FB(-20, m_encoder_actuator.GetPosition()));
		} else {
			m_actuator.Set(0.0);
		}

		/*

		 */
		if (otherStick->GetRawButton(1)) {
			//lift - south is up, north is down
			if (otherStick->GetY() * cabs(otherStick->GetY()) > 0) {
				//up
				m_lift.Set(0.7 * otherStick->GetY() * cabs(otherStick->GetY()));
				m_lift2.Set(
						0.7 * otherStick->GetY() * cabs(otherStick->GetY()));
			} else {
				if (m_encoder_lift.GetPosition() > 0) {
					//down
					m_lift.Set(
							0.25 * otherStick->GetY()
									* cabs(otherStick->GetY()));
					m_lift2.Set(
							0.25 * otherStick->GetY()
									* cabs(otherStick->GetY()));
				} else {
					m_lift.Set(0.0);
					m_lift2.Set(0.0);
				}
			}
		} else if (auxStick->GetRawButton(7)) {
			double vy = m_liftMotion.FB(160, m_encoder_lift.GetPosition());
			double factor = (vy > 0) ? 0.25 : 0.7;
			m_lift.Set(vy * factor);
			m_lift2.Set(vy * factor);
		} else if (auxStick->GetRawButton(8)) {
			double vy = m_liftMotion.FB(130, m_encoder_lift.GetPosition());
			double factor = (vy > 0) ? 0.25 : 0.7;
			m_lift.Set(vy * factor);
			m_lift2.Set(vy * factor);
		} else if (auxStick->GetRawButton(9)) {
			double vy = m_liftMotion.FB(61, m_encoder_lift.GetPosition());
			double factor = (vy > 0) ? 0.25 : 0.7;
			m_lift.Set(vy * factor);
			m_lift2.Set(vy * factor);
		} else if (auxStick->GetRawButton(10)) {
			double vy = m_liftMotion.FB(50, m_encoder_lift.GetPosition());
			double factor = (vy > 0) ? 0.25 : 0.7;
			m_lift.Set(vy * factor);
			m_lift2.Set(vy * factor);
		} else if (auxStick->GetRawButton(11)) {
			double vy = m_liftMotion.FB(17.5, m_encoder_lift.GetPosition());
			double factor = (vy > 0) ? 0.25 : 0.7;
			m_lift.Set(vy * factor);
			m_lift2.Set(vy * factor);
		} else if (auxStick->GetRawButton(12)) {
			double vy = m_liftMotion.FB(0, m_encoder_lift.GetPosition());
			double factor = (vy > 0) ? 0.25 : 0.7;
			m_lift.Set(vy * factor);
			m_lift2.Set(vy * factor);
		} else {
			m_lift.Set(0.0);
			m_lift2.Set(0.0);
		}

		if (otherStick->GetRawButton(5)) {
			//out
			srx_wheel_left.Set(ControlMode::PercentOutput, 1.0);
			srx_wheel_right.Set(ControlMode::PercentOutput, 1.0);
		} else if (otherStick->GetRawButton(3)) {
			//in
			srx_wheel_left.Set(ControlMode::PercentOutput, -1.0);
			srx_wheel_right.Set(ControlMode::PercentOutput, -1.0);
		} else {
			srx_wheel_left.Set(ControlMode::PercentOutput, 0.0);
			srx_wheel_right.Set(ControlMode::PercentOutput, 0.0);
		}

		if (otherStick->GetRawButton(6)) {
			//down
			m_tilt.Set(otherStick->GetY() * cabs(otherStick->GetY()));
			//m_tilt.Set(1.0);
			//} else if (otherStick->GetRawButton(6)) {
			//up
			//m_tilt.Set(-1.0);
		} else {
			m_tilt.Set(0.0);
		}

		frc::SmartDashboard::PutNumber("Actuator",
				m_actuator.GetEncoder().GetPosition());

		/*if(driveStickLeft->GetRawButton(1))
		 {
		 m_actuator.GetEncoder().SetPosition(0);
		 }*/
		/*
		 if (driveStickLeft->GetRawButton(2)) {
		 try {
		 autoMode = new AutonomousModeNewMotion(4096,4096,100,true);
		 } catch (std::bad_alloc &e) {
		 wpi::errs() << "Bad Allocation of Autonomous Mode.\n";
		 wpi::errs().flush();
		 }
		 }
		 */

		//if (otherStick->GetRawButton(7)) pixy.setServos((u_int16_t) ((-otherStick->GetZ()+1.0)*500.0),(u_int16_t) ((-otherStick->GetY()+1.0)*500.0));
		if (driveStickLeft->GetRawButton(1)
				|| driveStickRight->GetRawButton(1)) {
			srx_left.Set(ControlMode::PercentOutput,
					0.35 * driveStickLeft->GetY()
							* cabs(driveStickLeft->GetY()));
			srx_right.Set(ControlMode::PercentOutput,
					0.35 * driveStickRight->GetY()
							* cabs(driveStickRight->GetY()));
			srx_left2.Set(ControlMode::PercentOutput,
					0.35 * driveStickLeft->GetY()
							* cabs(driveStickLeft->GetY()));
			srx_right2.Set(ControlMode::PercentOutput,
					0.35 * driveStickRight->GetY()
							* cabs(driveStickRight->GetY()));
		} else if (driveStickLeft->GetRawButton(3)
				|| driveStickRight->GetRawButton(3)) {
			srx_left.Set(ControlMode::PercentOutput, -0.1);
			srx_left2.Set(ControlMode::PercentOutput, -0.1);
			srx_right.Set(ControlMode::PercentOutput, -0.1);
			srx_right2.Set(ControlMode::PercentOutput, -0.1);
		} else if (driveStickLeft->GetRawButton(2)
				|| driveStickRight->GetRawButton(2)) {
			srx_left.Set(ControlMode::PercentOutput,
					0.90 * driveStickLeft->GetY()
							* cabs(driveStickLeft->GetY()));
			srx_right.Set(ControlMode::PercentOutput,
					0.90 * driveStickRight->GetY()
							* cabs(driveStickRight->GetY()));
			srx_left2.Set(ControlMode::PercentOutput,
					0.90 * driveStickLeft->GetY()
							* cabs(driveStickLeft->GetY()));
			srx_right2.Set(ControlMode::PercentOutput,
					0.90 * driveStickRight->GetY()
							* cabs(driveStickRight->GetY()));
		} else {
			srx_left.Set(ControlMode::PercentOutput,
					0.65 * driveStickLeft->GetY()
							* cabs(driveStickLeft->GetY()));
			srx_right.Set(ControlMode::PercentOutput,
					0.65 * driveStickRight->GetY()
							* cabs(driveStickRight->GetY()));
			srx_left2.Set(ControlMode::PercentOutput,
					0.65 * driveStickLeft->GetY()
							* cabs(driveStickLeft->GetY()));
			srx_right2.Set(ControlMode::PercentOutput,
					0.65 * driveStickRight->GetY()
							* cabs(driveStickRight->GetY()));
		}

		/*		 if (driveStickRight->GetRawButton(4)) {
		 this->tryLines = true;
		 if (this->tryLines) {
		 int8_t featureResult = pixy.line.getMainFeatures(LINE_VECTOR, false);
		 //wpi::errs() << "\nPixy Return " << (uint16_t)featureResult;

		 if (featureResult >= 0) {
		 frc::SmartDashboard::PutBoolean("DB/LED 0", true);
		 }

		 if (featureResult >= 1) {
		 Vector v = pixy.line.vectors[0];
		 wpi::errs() << "\n" << (uint16_t) v.m_x0 << ", "
		 << (uint16_t) v.m_y0 << "; " << (uint16_t) v.m_x1
		 << ", " << (uint16_t) v.m_y1;
		 frc::SmartDashboard::PutBoolean("DB/LED 1", true);
		 if (v.m_x0 > 42) {
		 wpi::errs() << "\nTrying to turn right";
		 srx_left.Set(ControlMode::PercentOutput, -0.33);
		 srx_left2.Set(ControlMode::PercentOutput, -0.33);
		 srx_right.Set(ControlMode::PercentOutput, 0.33);
		 srx_right2.Set(ControlMode::PercentOutput, 0.33);
		 }
		 if (v.m_x0 < 36) {
		 wpi::errs() << "\nTrying to turn left";
		 srx_left.Set(ControlMode::PercentOutput, 0.33);
		 srx_left2.Set(ControlMode::PercentOutput, 0.33);
		 srx_right.Set(ControlMode::PercentOutput, -0.33);
		 srx_right2.Set(ControlMode::PercentOutput, -0.33);
		 }
		 if (v.m_x0 >= 36 && v.m_x0 <= 42) {
		 wpi::errs() << "\nCenter";
		 if ((v.m_y0 < 49)
		 || ((v.m_y0 >= 49) && (v.m_y1 < 30))) {
		 wpi::errs() << "\nDrive";
		 srx_left.Set(ControlMode::PercentOutput, -0.0);
		 srx_right.Set(ControlMode::PercentOutput, -0.0);
		 srx_left2.Set(ControlMode::PercentOutput, -0.0);
		 srx_right2.Set(ControlMode::PercentOutput, -0.0);
		 }
		 }
		 } else {
		 frc::SmartDashboard::PutBoolean("DB/LED 1", false);
		 }

		 if (featureResult == 0) {
		 srx_left.Set(ControlMode::PercentOutput, 0);
		 srx_right.Set(ControlMode::PercentOutput, 0);
		 srx_left2.Set(ControlMode::PercentOutput, 0);
		 srx_right2.Set(ControlMode::PercentOutput, 0);
		 }

		 if (featureResult == PIXY_RESULT_ERROR) {
		 frc::SmartDashboard::PutBoolean("DB/LED 0", false);
		 pixy.reset();
		 }
		 } else {
		 srx_left.Set(ControlMode::PercentOutput, 0.0);
		 srx_right.Set(ControlMode::PercentOutput, 0.0);
		 srx_left2.Set(ControlMode::PercentOutput, 0.0);
		 srx_right2.Set(ControlMode::PercentOutput, 0.0);
		 }
		 //only query the camera every *other* cycle
		 this->tryLines = !(this->tryLines);
		 } else {
		 frc::SmartDashboard::PutBoolean("DB/LED 0", false);
		 frc::SmartDashboard::PutBoolean("DB/LED 1", false);

		 //original drive code goes here

		 }

		 }
		 */

		if (driveStickLeft->GetRawButton(4)) {
			pixy.reset(true);
			frc::SmartDashboard::PutBoolean("DB/LED 0", false);
		}

		//pixy.ccc.getBlocks(false,true,255);
		//for (int i=0;i<pixy.ccc.numBlocks;i++) {
		//wpi::errs() << "Found " << pixy.ccc.blocks[i].m_x << ", " << pixy.ccc.blocks[i].m_y;
		//}

		/*if (driveStickRight->GetRawButton(12)) {
		 //srx_left.GetSensorCollection().SetQuadraturePosition(0);
		 //srx_right.GetSensorCollection().SetQuadraturePosition(0);
		 }*/

		frc::SmartDashboard::PutNumber("Encoder Tilt",
				m_encoder_tilt.GetPosition());
		frc::SmartDashboard::PutNumber("Encoder Lift",
				m_encoder_lift.GetPosition());
		frc::SmartDashboard::PutNumber("Encoder Lift 2",
				m_encoder_lift2.GetPosition());
		frc::SmartDashboard::PutNumber("Encoder Actuator",
				m_encoder_actuator.GetPosition());
		frc::SmartDashboard::PutNumber("Encoder Carriage",
				m_encoder_carriage.GetPosition());

		frc::SmartDashboard::PutNumber("Left",
				srx_left.GetSelectedSensorPosition());//GetSensorCollection().GetQuadraturePosition());
		frc::SmartDashboard::PutNumber("Right",
				srx_right.GetSelectedSensorPosition());	//GetSensorCollection().GetQuadraturePosition());
		frc::SmartDashboard::PutNumber("Left Range", rangeleft->GetVoltage());
		frc::SmartDashboard::PutNumber("Right Range", rangeright->GetVoltage());

	}
}

void Robot::DisabledInit() {

}

void Robot::DisabledPeriodic() {

}

void Robot::TestPeriodic() {
}

#ifndef RUNNING_FRC_TESTS
int main() {
	return frc::StartRobot<Robot>();
}
#endif

void Robot::EncoderSave() {

}

void Robot::EncoderLoad() {

}
