package com.thereferees.ftc_app.OpModes.lib;

import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.swerverobotics.library.interfaces.IBNO055IMU;

/**
 * Created by idean on 1/29/16.
 */
public class GyroPIDControllerNew extends PIDController {
    private final double FINE_TUNE_START_POWER = 0.4;
    private final double POWER_MIN = 0.20;
    int sign = 1;
    private double powerDescentFast;
    private double powerDescentSlow;
    private IBNO055IMU gyro;
    public GyroPIDControllerNew(IBNO055IMU gyro, double threshold, double slowDownStart, double fineTuneStart, DcMotor... motors) {
        super(threshold, motors);
        this.gyro = gyro;

        this.fineTuneStart = fineTuneStart;
        this.slowDownStart = slowDownStart;
        super.setSides(1);
        /*
        powerDescentFast = (slowDownStart - fineTuneStart) / (1 - FINE_TUNE_START_POWER);
        powerDescentSlow = (slowDownStart) / (FINE_TUNE_START_POWER - POWER_MIN);
        */
        powerDescentFast = 1d / 60;
        powerDescentSlow = 0.25 / 15;
    }

    @Override
    public void setTargets(double target) {
        final double currentPosition = getCurrentPosition()[0];
        targets[0] = (currentPosition + target) % 360;
    }

    @Override
    public double[] run() {
        double[] power = new double[2];
        if (!hasReachedDestination()) {
            final double currVal = getCurrentPosition()[0];
            double error = targets[0] - currVal;

            if (Math.abs(error) > 180) {
                error += 360;
            }

            if (Math.abs(error) >= threshold) {
                double powerMultiplier = (Math.abs(error) >= fineTuneStart) ? powerDescentFast : powerDescentSlow;
                power[0] = -error * powerMultiplier;
            }
            power[0] = Range.clip(power[0], -powerMax, powerMax);
            power[1] = -power[0];
            Log.d("Gyro1", "POWER 0: " + power[0]);
            Log.d("Gyro1", "POWER 1: " + power[1]);
        }
        return power;
    }

    @Override
    public double[] getCurrentPosition() {
        double[] position = new double[1];
        double heading = gyro.getAngularOrientation().heading;
        position[0] = heading;
        return position;
    }
}
