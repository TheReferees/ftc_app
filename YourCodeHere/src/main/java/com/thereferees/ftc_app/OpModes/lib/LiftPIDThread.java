package com.thereferees.ftc_app.OpModes.lib;

import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Andrew on 11/19/2015.
 */
public class LiftPIDThread extends PIDThread {

    public LiftPIDThread(double kP, double kI, double kD, DcMotor liftMotor) {
        super(kP, kI, kD, liftMotor);
    }

    public void setTarget(int target) {
        this.target = target;
        motorTargets[0] = motors[0].getCurrentPosition() + target;
    }
}
