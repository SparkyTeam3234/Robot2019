/*
 * NewMotion.h
 *
 *  Created on: Mar 13, 2019
 *      Author: user
 */

#pragma once
#include <ctre/Phoenix.h>

using namespace std;

void initializeTalon(TalonSRX *talon) {
	/* Set relevant frame periods to be at least as fast as periodic rate */
    talon->SetStatusFramePeriod(StatusFrameEnhanced::Status_13_Base_PIDF0, 10, 10);
    talon->SetStatusFramePeriod(StatusFrameEnhanced::Status_10_MotionMagic, 10, 10);

    /* Set the peak and nominal outputs */
    talon->ConfigNominalOutputForward(0, 10);
    talon->ConfigNominalOutputReverse(0, 10);
    talon->ConfigPeakOutputForward(1, 10);
    talon->ConfigPeakOutputReverse(-1, 10);

    /* Set Motion Magic gains in slot0 - see documentation */
    talon->SelectProfileSlot(0, 0);
    talon->Config_kF(0, 0.3, 10);
    talon->Config_kP(0, 0.1, 10);
    talon->Config_kI(0, 0.0, 10);
    talon->Config_kD(0, 0.0, 10);

    /* Set acceleration and vcruise velocity - see documentation */
    talon->ConfigMotionCruiseVelocity(1500, 10);
    talon->ConfigMotionAcceleration(1500, 10);
}
