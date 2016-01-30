package com.thereferees.ftc_app.OpModes.lib;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Andrew on 1/7/2016.
 */
public class PIDController {

    // variables that change depending on part of robot
    private double wheelDiameter;
    private int gearRatio;
    private int threshold;
    private int sides = 1;
    private double slowDownStart;
    private double fineTuneStart;
    private double powerMin;
    private double powerMax = 1;
    final int TICKS_PER_REVOLUTION   = 1120;
    private double conversionFactor;
    private DcMotor[] motors;
    private int[] targets;
    private double turnDiameter;


    public PIDController(double wheelDiameter, int gearRatio, int threshold, double slowDownStart, double fineTuneStart, double powerMin, TypePID typePID, DcMotor ... motors) {
        this(wheelDiameter, 1.0d, gearRatio, threshold, slowDownStart, fineTuneStart, powerMin, typePID, motors);
    }

    /**
     * @param wheelDiameter
     * @param turnDiameter
     * @param gearRatio
     * @param threshold Amount of allowable error in number of encoder ticks to destination.
     * @param slowDownStart Number of wheel revolutions before slowing down the first time.
     * @param fineTuneStart Number of wheel revolutions before slowing down the second time.
     * @param powerMin Minimum motor power.
     * @param typePID Determines conversion factor (degrees to ticks or distance to ticks based off type)
     * @param motors
     */
    public PIDController(double wheelDiameter, double turnDiameter, int gearRatio, int threshold, double slowDownStart, double fineTuneStart, double powerMin, TypePID typePID, DcMotor ... motors) {
        this.wheelDiameter = wheelDiameter;
        this.turnDiameter = turnDiameter;
        this.gearRatio = gearRatio;
        this.threshold = threshold;
        this.slowDownStart = slowDownStart;
        this.fineTuneStart = fineTuneStart;
        this.powerMin = powerMin;
        this.typePID = typePID;
        switch (typePID) {
            case DRIVE:
                conversionFactor = TICKS_PER_REVOLUTION / (wheelDiameter * Math.PI * gearRatio);
                break;
            case TURN:
                conversionFactor = turnDiameter * TICKS_PER_REVOLUTION / (wheelDiameter * gearRatio * 360);
                break;
            case LIFT:
                break;
            default:
                break;
        }
        this.motors = new DcMotor[motors.length];
        for(int i = 0; i < this.motors.length; i++) {
            this.motors[i] = motors[i];
        }
        boolean isSymmetrical = (motors.length % 2 == 0);
        if(isSymmetrical) {
            sides = 2;
        } else {
            sides = 1;
        }
        targets = new int[sides];
    }
    public enum TypePID {
        DRIVE,
        TURN,
        LIFT
    } TypePID typePID;

    public void setPowerMax(double powerMax) {
        this.powerMax = powerMax;
    }


    //Turns left with positive value
    public void setTargets(double target) {
        if (typePID == TypePID.TURN)
            target *= 1.166667;
        for(int i = 0; i < sides; i++) {
            targets[i] = (int) (getCurrentPosition()[i] + target * conversionFactor);

            if (typePID == TypePID.TURN) {
                target *= -1;
            }
        }
    }

    public int[] getTargets() {
        return targets;
    }

    public double[] run() {
        final double K_FAST              = 1 / (slowDownStart * TICKS_PER_REVOLUTION);
        final double K_SLOW              = (1 / slowDownStart - powerMin / fineTuneStart) / TICKS_PER_REVOLUTION;
        final double TICK_OFFSET         = powerMin * slowDownStart * fineTuneStart * TICKS_PER_REVOLUTION / (fineTuneStart - powerMin * slowDownStart);

        int[] currVal       = new int[sides];
        int[] error         = new int[sides];
        double[] power      = new double[sides];

        if(!hasReachedDestination()) {
            for(int i = 0; i < sides; i++) {
                currVal[i] = getCurrentPosition()[i];
                error[i] = targets[i] - currVal[i];
                if(Math.abs(error[i]) < threshold) {
                    power[i] = 0.0d;
                } else {
                    // try using - fineTuneStart on K_SLOW, maybe need /2
                    power[i] = (Math.abs(error[i]) > fineTuneStart * TICKS_PER_REVOLUTION) ? K_FAST * Math.abs(error[i]) : K_SLOW * (TICK_OFFSET + Math.abs(error[i]));
                }
                if(error[i] < 0.0d) {
                    power[i] *= -1;
                }
                power[i] = Range.clip(power[i], -powerMax, powerMax);
            }
        }
        return power;
    }

    public boolean hasReachedDestination() {
        for(int i = 0; i < sides; i++) {
            if(Math.abs(targets[i] - getCurrentPosition()[i]) > threshold) {
                return false;
            }
        }
        for (DcMotor motor : motors) {
            motor.setPower(0.0d);
        }
        return true;
    }

    public int[] getCurrentPosition() {
        int[] temp = new int[sides];
        for(int i = 0; i < sides; i++) {
            int sidePos = 0;
            for (int j = 0; j < motors.length / sides; j++) {
                sidePos += motors[i * 2 + j].getCurrentPosition();
            }
            temp[i] = sidePos / (int) (motors.length / sides);
        }

        return temp;
    }

    void stop() {
        for(DcMotor motor : motors) {
            motor.setPower(0.0d);
        }
    }
}
