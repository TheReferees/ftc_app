package com.thereferees.ftc_app.OpModes.lib;

import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by hoangam on 11/19/2015.
 */
public class DrivePIDThread extends PIDThread {
    final private double INCHES_TO_TICKS = 1.0d;
    public DrivePIDThread(double kP, double kI, double kD, DcMotor driveFR, DcMotor driveFL, DcMotor driveBR, DcMotor driveBL) {
        super(kP, kI, kD, driveFR, driveFL, driveBR, driveBL);
    }
    //////// need to do!!!!!!!!!!!!!!!!! ///////////////////
    // convert ticks to inches
    // find average of 5 trials, ticks per 18 inches by driving
    // target is entered in inches
    public void setTarget(double target) {
        this.target = target * INCHES_TO_TICKS;
        for(int i = 0; i < motorTargets.length; i++) {
            motorTargets[i] = (int)(motors[i].getCurrentPosition() + target);
        }
    }
}
