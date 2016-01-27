package com.thereferees.ftc_app.OpModes.lib;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import java.util.Arrays;


/**
 * Created by Andrew on 11/18/2015.
 */
public class PIDThread extends Thread implements PIDInterface{
    protected double[] kP, kI, kD, PIDValue, powers;
    protected int[] accumError;
    protected double    target   = 0.0d;
    protected double    maxPower = 1.0d,
                        minPower = -1.0d;
    protected int       thresholdError = 40;
    protected double    thresholdPower = 0.1d;
    protected double actualPIDValue;
    DcMotor motors[];
    int motorTargets[];

    PIDThread(double kP, double kI, double kD, DcMotor ... motors) {
        this.kP = new double[motors.length];
        this.kI = new double[motors.length];
        this.kD = new double[motors.length];
        this.PIDValue = new double[motors.length];
        this.accumError = new int[motors.length];

        this.powers = new double[motors.length];
        this.motors = motors;
        this.motorTargets = new int[motors.length];
        // setting default array values
        Arrays.fill(this.kP, kP);
        Arrays.fill(this.kI, kI);
        Arrays.fill(this.kD, kD);
        Arrays.fill(accumError, 0);
        Arrays.fill(motorTargets, 0);
        Arrays.fill(PIDValue, 0.0d);
        Arrays.fill(powers, 0.0d);
    }

    public void setTarget(double target) {
        this.target = target;
        for(int i = 0; i < motorTargets.length; i++) {
            // averages current values and adds target to get new motor targets
            motorTargets[i] = (int)(motors[i].getCurrentPosition() + target);
        }
    }

    protected boolean didReachTarget() {
        for(int i = 0; i < motors.length; i++) {
            if(Math.abs(Math.abs(motors[i].getCurrentPosition()) - Math.abs(motorTargets[i])) > thresholdError) {
                return false;
            }
        }
        return true;
    }
    public double getPIDValue(int motor) {
        //return kP[motor] * (motorTargets[motor] - motors[motor].getCurrentPosition());
        return kP[motor] * (motorTargets[motor] - motors[motor].getCurrentPosition());
    }
    public int getMotorTarget(int motor) {
        return motorTargets[motor];
    }
    public double getKP(int motor) {
        return kP[motor];
    }
    public double getCurrError(int motor) {
        return motorTargets[motor] - motors[motor].getCurrentPosition();
    }
    public double getCurrPos(int motor) {
        return motors[motor].getCurrentPosition();
    }
    public double getSupposedPower(int motor) {
        return Range.clip(kP[motor] * (motorTargets[motor] - motors[motor].getCurrentPosition()), -1.0d, 1.0d);
    }

    public double[] getPowers() {
        return powers;
    }
    public void setMaxPower(double maxPower) {
        this.maxPower = maxPower;
    }
    public void setMinPower(double minPower) {
        this.minPower = minPower;
    }

    @Override
    public void run() {
        try {
            while(!Thread.currentThread().isInterrupted()) {
                if(!didReachTarget()) {
                    for (int i = 0; i < motors.length; i++) {
                        int error = motorTargets[i] - motors[i].getCurrentPosition();
                        PIDValue[i] = kP[i] * error;
                        //accumError[i] += error;
                        actualPIDValue = kP[i] * error;
                        //motors[i].setPower(Range.clip(actualPIDValue, -1.0d, 1.0d));
                        if (Math.abs(Range.clip(actualPIDValue, -1.0d, 1.0d)) > thresholdPower) {
                            motors[i].setPower(Range.clip(actualPIDValue, -1.0d, 1.0d));
                            //powers[i] = Range.clip(actualPIDValue, -1.0d, 1.0d);
                            //motors[i].setPower(0.0d);
                        } else {
                            motors[i].setPower(0.0d);
                            //powers[i] = 0.0d;
                            //powers[i] = Range.clip(actualPIDValue, -1.0d, 1.0d);
                        }
                        //powers[i] = Range.clip(actualPIDValue, -1.0d, 1.0d);
                        //motors[i].setPower(Range.clip(actualPIDValue, -1.0d, 1.0d));
                    }
                }
                sleep(10);
            }
        } catch(InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
}