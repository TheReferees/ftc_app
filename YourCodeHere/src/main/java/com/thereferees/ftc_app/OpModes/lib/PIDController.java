package com.thereferees.ftc_app.OpModes.lib;

import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by idean on 1/29/16.
 */
public abstract class PIDController {
    protected double wheelDiameter;
    protected int gearRatio;
    protected double threshold;
    protected int sides = 1;
    private double conversionFactor = 1.0d;
    protected double slowDownStart;
    protected double fineTuneStart;
    protected double powerMin;
    protected double powerMax = 1;
    protected DcMotor[] motors;
    protected double[] targets;

    public enum TypePID {
        DRIVE,
        TURN,
        LIFT
    } TypePID typePID;

    public PIDController(double threshold, DcMotor... motors) {
        this.motors = new DcMotor[motors.length];
        for(int i = 0; i < this.motors.length; i++) {
            this.motors[i] = motors[i];
        }
        this.threshold = threshold;
    }

    public double[] getTargets() {
        return targets;
    }

    public void setConversionFactor(double conversionFactor) {
        this.conversionFactor = conversionFactor;
    }

    public void setSides(int sides) {
        this.sides = sides;
        this.targets = new double[sides];
    }

    public void setTargets(double target) {
        for(int i = 0; i < sides; i++) {
            Log.d("SetTargets1", "" + conversionFactor);
            targets[i] = (int) (getCurrentPosition()[i] + target * conversionFactor);
        }
    }

    public boolean hasReachedDestination() {
        for(int i = 0; i < targets.length; i++) {
            if(Math.abs(targets[i] - getCurrentPosition()[i]) >= threshold) {
                return false;
            }
        }
        stopMotors();
        return true;
    }

    void stopMotors() {
        for(DcMotor motor : motors) {
            motor.setPower(0.0d);
        }
    }

    public abstract double[] getCurrentPosition();
    public abstract double[] run();
}
