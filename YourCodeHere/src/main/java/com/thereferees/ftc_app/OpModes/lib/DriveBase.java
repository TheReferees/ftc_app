package com.thereferees.ftc_app.OpModes.lib;

import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Andrew on 1/8/16.
 */
public class DriveBase extends MotorComponent {
    public DriveBase() {
        this(0, 0, Direction.FORWARD);
    }
    public DriveBase(int wheelDiameter, int gearRatio, Direction direction) {
        setWheelDiameter(wheelDiameter);
        setGearRatio(gearRatio);
        setDirection(direction);
    }
    public DriveBase(int wheelDiameter, int gearRatio, Direction direction, int threshold, double slowDownStart, double fineTuneStart, double powerMin, double conversionFactor, DcMotor ... motors) {
        this(wheelDiameter, gearRatio, direction);
        //driveBasePID = new PIDController(wheelDiameter, gearRatio, threshold, slowDownStart, fineTuneStart, powerMin, conversionFactor, motors);
    }
    double angleConversionFactor;
    double orientation;
    private PIDController driveBasePID;
    public void setMotors(DcMotor... motors) {
        this.setMotors(motors);
    }
    public void setAngleConversionFactor(double angleConversionFactor) {
        this.angleConversionFactor = angleConversionFactor;
    }
    void updateOrientation() {
        orientation = (getPositions()[0] - getPositions()[1]) * angleConversionFactor;
    }
    void update() {

    }
}
