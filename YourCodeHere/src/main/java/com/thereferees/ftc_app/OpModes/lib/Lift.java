package com.thereferees.ftc_app.OpModes.lib;

import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Andrew on 1/7/2016.
 */
public class Lift extends MotorComponent {
    public Lift() {
        this(0, 0, Direction.FORWARD);
    }
    public Lift(int wheelDiameter, int gearRatio, Direction direction) {
        this(wheelDiameter, gearRatio, 0, 0.0d, 0.0d, 0.0d, 0.0d, direction);
    }
    public Lift(int wheelDiameter, int gearRatio, int threshold, double slowDownStart, double fineTuneStart, double powerMin, double conversionFactor, DcMotor ... motors) {
        this(wheelDiameter, gearRatio, threshold, slowDownStart, fineTuneStart, powerMin, conversionFactor, Direction.FORWARD, motors);
    }
    public Lift(int wheelDiameter, int gearRatio, int threshold, double slowDownStart, double fineTuneStart, double powerMin, double conversionFactor, Direction direction, DcMotor ... motors) {
        setWheelDiameter(wheelDiameter);
        setGearRatio(gearRatio);
        setDirection(direction);
        //liftPID = new PIDController(wheelDiameter, gearRatio, threshold, slowDownStart, fineTuneStart, powerMin, conversionFactor, motors);
    }
    private PIDController liftPID;

    public void setMotors(DcMotor ... motors) {
        this.setMotors(motors);
    }
}
