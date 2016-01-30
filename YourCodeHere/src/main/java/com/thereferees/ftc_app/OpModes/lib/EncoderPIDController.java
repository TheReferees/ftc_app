package com.thereferees.ftc_app.OpModes.lib;

import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Andrew on 1/7/2016.
 */
public class EncoderPIDController extends PIDController {

    // variables that change depending on part of robot
    final int TICKS_PER_REVOLUTION   = 1120;
    private double conversionFactor;
    private double turnDiameter;

    private final double POWER_MIN = 0.25;

    /**
     * @param wheelDiameter
     * @param gearRatio
     * @param threshold Amount of allowable error in number of encoder ticks to destination.
     * @param slowDownStart Number of wheel revolutions before slowing down the first time.
     * @param fineTuneStart Number of wheel revolutions before slowing down the second time.
     * @param typePID Determines conversion factor (degrees to ticks or distance to ticks based off type)
     * @param motors
     */
    public EncoderPIDController(double wheelDiameter, int gearRatio, double threshold, double slowDownStart, double fineTuneStart, TypePID typePID, DcMotor ... motors) {
        super(threshold, motors);
        boolean isSymmetrical = (motors.length % 2 == 0);
        int sides = 1;
        if(isSymmetrical) {
            sides = 2;
        }
        super.setSides(sides);
        this.wheelDiameter = wheelDiameter;
        this.gearRatio = gearRatio;
        this.slowDownStart = slowDownStart;
        this.fineTuneStart = fineTuneStart;
        this.typePID = typePID;
        switch (typePID) {
            case DRIVE:
                super.setConversionFactor(TICKS_PER_REVOLUTION / (wheelDiameter * Math.PI * gearRatio));
                Log.d("SetTargets", "" + conversionFactor);
                break;
            case LIFT:
                break;
            default:
                break;
        }
        targets = new double[sides];
    }


    public void setPowerMax(double powerMax) {
        this.powerMax = powerMax;
    }

    public double[] run() {
        final double K_FAST              = 1 / (slowDownStart * TICKS_PER_REVOLUTION);
        final double K_SLOW              = (1 / slowDownStart - POWER_MIN / fineTuneStart) / TICKS_PER_REVOLUTION;
        final double TICK_OFFSET         = POWER_MIN * slowDownStart * fineTuneStart * TICKS_PER_REVOLUTION / (fineTuneStart - POWER_MIN * slowDownStart);

        double[] currVal       = new double[sides];
        double[] error         = new double[sides];
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

    public double[] getCurrentPosition() {
        double[] temp = new double[sides];
        for(int i = 0; i < targets.length; i++) {
            int sidePos = 0;
            for (int j = 0; j < motors.length / sides; j++) {
                sidePos += motors[i * 2 + j].getCurrentPosition();
            }
            temp[i] = sidePos / (int) (motors.length / sides);
        }

        return temp;
    }
}
