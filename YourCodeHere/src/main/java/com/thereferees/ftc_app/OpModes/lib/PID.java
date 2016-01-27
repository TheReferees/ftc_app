package com.thereferees.ftc_app.OpModes.lib;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Andrew on 10/31/2015.
 */
public abstract class PID  implements  Runnable{
    protected float kP,
                    kI,
                    kD;
    protected float maxPower,
                    minPower,
                    minPIDPower;
    protected int acceptableError;  // in encoder ticks
    protected float target,           // in inches
                    distanceTravelled;    // in encoder ticks
    private boolean isMoving = true,
                    isFineTune = false;
    private double  timer = 0.0f,   // in milliseconds
                    fineTuneTimer = 0.0f,
                    currDt = 0.0f;
    private float   currError = 0.0f,   // in encoder ticks
                    prevError = 0.0f,   // in encoder ticks
                    errorRate = 0.0f,   // in encoder ticks
                    accumError = 0.0f;  // in encoder ticks
    private float   PIDValue = 0.0f,
                    power = 0.0f;

    private final int PULSE_PER_REV = 1120; // encoder ticks per revolution
    protected double GEAR_RATIO, OBJECT_CIRCUMFERENCE; // in inches
    ElapsedTime clock;

    public void setTarget(float enteredTarget) {
        this.target = distanceTravelled + (float)(enteredTarget / GEAR_RATIO / OBJECT_CIRCUMFERENCE * PULSE_PER_REV);
    }

    @Override
    public void run() {
        clock.reset();
        clock.startTime();
        try {
            while(isMoving) {
                currDt = clock.time() / 1000.0d;
                clock.reset();
                prevError = currError;
                currError = target - distanceTravelled;
                accumError += currError;
                errorRate = (prevError - currError) / (float)currDt;
                PIDValue = kP * currError + kI * accumError + kD * errorRate;
                PIDValue = Range.clip(PIDValue, -maxPower, maxPower);
                power = PIDValue;
                if(currError < acceptableError) {
                    power = 0.0f;
                    isMoving = false;
                }
                Thread.sleep(10);
                // add in fine tune mode later
            }
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
            return;
        }
    }
}
