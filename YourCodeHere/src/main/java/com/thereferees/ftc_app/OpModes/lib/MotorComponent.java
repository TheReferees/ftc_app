package com.thereferees.ftc_app.OpModes.lib;

import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Andrew on 1/7/2016.
 */
public abstract class MotorComponent {
    protected final int TICKS_PER_REVOLUTION = 1120;
    protected int wheelDiameter = 1;
    protected int gearRatio = 1;
    protected double conversionFactor = 1.0d;
    protected boolean isSymmetrical = false;
    protected DcMotor motors[];
    protected int positions[];
    protected enum Direction {
        FORWARD,
        REVERSE
    } Direction direction = Direction.FORWARD;
    enum Axis {
        X,
        Y
    }
    void updatePosition() {
        if(isSymmetrical) {
            for(int i = 0; i < 2; i++) {
                int sideVal = 0;
                for(int j = 0; j < motors.length / 2; j++) {
                    sideVal += motors[i * j].getCurrentPosition();
                }
                positions[i] = sideVal / (int)(motors.length / 2);
            }
        } else {
            positions[0] = motors[0].getCurrentPosition();
        }
    };

    void setWheelDiameter(int wheelDiameter) {
        this.wheelDiameter = wheelDiameter;
    }
    void setGearRatio(int gearRatio) {
        this.gearRatio = gearRatio;
    }
    void setDirection(Direction direction) {
        this.direction = direction;
    }
    void setMotors(DcMotor ... motors) {
        isSymmetrical = (motors.length % 2 == 0);
        positions = (isSymmetrical) ? new int[2] : new int[1];
        this.motors = new DcMotor[motors.length];
        for(int i = 0; i < this.motors.length; i++) {
            this.motors[i] = motors[i];
        }
    }
    int getWheelDiameter() {
        return  wheelDiameter;
    }
    int getGearRatio() {
        return gearRatio;
    }
    Direction getDirection() {
        return direction;
    }
    DcMotor[] getMotors() {
        return motors;
    }
    int[] getPositions() {
        return positions;
    }
}
