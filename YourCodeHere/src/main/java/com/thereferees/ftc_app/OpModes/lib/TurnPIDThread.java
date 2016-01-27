package com.thereferees.ftc_app.OpModes.lib;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Andrew on 11/19/2015.
 */
public class TurnPIDThread extends PIDThread {
    final private double DEGREES_TO_TICKS = 1.0d;
    public TurnPIDThread(double kP, double kI, double kD, DcMotor driveFR, DcMotor driveFL, DcMotor driveBR, DcMotor driveBL) {
        super(kP, kI, kD, driveFR, driveFL, driveBR, driveBL);
    }
    //////// need to do!!!!!!!!!!!!!!!!! ///////////////////
    // convert degrees to ticks
    // find average of 5 trials, ticks per 90 inches by driving
    // target is entered in degrees
    public void setTarget(double target) {
        this.target = target * DEGREES_TO_TICKS;
        motorTargets[0] = (int)((motors[0].getCurrentPosition() + motors[2].getCurrentPosition()) / 2 + this.target);
        motorTargets[1] = (int)((motors[1].getCurrentPosition() + motors[3].getCurrentPosition()) / 2 - this.target);
        motorTargets[2] = (int)((motors[0].getCurrentPosition() + motors[2].getCurrentPosition()) / 2 + this.target);
        motorTargets[3] = (int)((motors[1].getCurrentPosition() + motors[3].getCurrentPosition()) / 2 - this.target);
    }
    @Override
    public void run() {
        try {
            double power = 0.0d;
            while(!Thread.currentThread().isInterrupted()) {
                if(!didReachTarget()) {
                    for (int i = 0; i < 2; i++) {
                        int error = (int)Math.ceil((motorTargets[i] + motorTargets[i + 2] - motors[i].getCurrentPosition() - motors[i + 2].getCurrentPosition()) / 2);
                        PIDValue[i] = kP[i] * error;
                        PIDValue[i + 2] = kP[i + 2] * error;
                        accumError[i] += error;
                        actualPIDValue = kP[i] * error + kI[i] * accumError[i];
                        //motors[i].setPower(Range.clip(actualPIDValue, -1.0d, 1.0d));
                        if (Math.abs(Range.clip(actualPIDValue, -1.0d, 1.0d)) > thresholdPower) {
                            power = Range.clip(actualPIDValue, -1.0d, 1.0d);
                        }
                        powers[i + 2] = power;
                        motors[i].setPower(power);
                        motors[i + 2].setPower(power);
                    }
                } else {
                    for (DcMotor x : motors) {
                        x.setPower(0.0d);
                    }
                }
                sleep(10);
            }
        } catch(InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
}
