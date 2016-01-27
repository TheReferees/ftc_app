package com.thereferees.ftc_app.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.thereferees.ftc_app.OpModes.lib.Action;
import com.thereferees.ftc_app.OpModes.lib.PIDController;

import org.swerverobotics.library.interfaces.Autonomous;

/**
 * A skeletal example of a do-nothing first OpMode. Go ahead and change this code
 * to suit your needs, or create sibling OpModes adjacent to this one in the same
 * Java package.
 */
@Autonomous(name="Auton")
public class Auton extends OpMode {
    public enum State {
        PUSHING_BUTTON, DROPPING_CLIMBERS, PARKING
    }

    public enum TurnState {
        FORWARD, TURNING_LEFT, TURNING_RIGHT
    }

    public enum PushButtonState {
        TURNING, PUSHING_BUTTON
    }

    DcMotor motorTopRight;
    DcMotor motorTopLeft;
    DcMotor motorBottomRight;
    DcMotor motorBottomLeft;

    Servo buttonPusher;

    private long actionStartTime;
    private State state;
    private TurnState turnState;
    private PushButtonState pushButtonState;

    final int       DRIVE_GEAR_RATIO        = 1,
                    DRIVE_THRESHOLD         = 20,
                    TURN_THRESHOLD          = 80;

    final double    DRIVE_WHEEL_DIAMETER    = 4,
                    DRIVE_SLOW_DOWN_START   = 1.5d,
                    DRIVE_FINE_TUNE_START   = 0.5d,
                    DRIVE_POWER_MIN         = 0.25d,
                    TURN_SLOW_DOWN_START    = 1.5d,
                    TURN_FINE_TUNE_START    = 0.5d,
                    TURN_POWER_MIN          = 0.25d,
                    TURN_DIAMETER           = Math.sqrt(Math.pow(9.0d, 2.0d) + Math.pow(16.5d, 2.0d));


    private PIDController drivePIDController;

    @Override
    public void init() {
        actionStartTime = 0;
        state = State.PUSHING_BUTTON;
        motorTopRight = hardwareMap.dcMotor.get("M_driveFR");
        motorTopLeft = hardwareMap.dcMotor.get("M_driveFL");
        motorBottomRight = hardwareMap.dcMotor.get("M_driveBR");
        motorBottomLeft = hardwareMap.dcMotor.get("M_driveBL");

        motorTopRight.setDirection(DcMotor.Direction.REVERSE);
        motorBottomRight.setDirection(DcMotor.Direction.REVERSE);

        //buttonPusher = hardwareMap.servo.get("buttonPusher");
        turnState = TurnState.FORWARD;
        pushButtonState = PushButtonState.TURNING;

        drivePIDController = new PIDController(DRIVE_WHEEL_DIAMETER, DRIVE_GEAR_RATIO, DRIVE_THRESHOLD, DRIVE_SLOW_DOWN_START, DRIVE_FINE_TUNE_START, DRIVE_POWER_MIN, PIDController.TypePID.DRIVE, motorTopRight, motorTopLeft, motorBottomLeft, motorBottomRight);
        turnPIDController = new PIDController(DRIVE_WHEEL_DIAMETER, TURN_WHEEL_DIAMETER, DRIVE_GEAR_RATIO, TURN_THRESHOLD, TURN_SLOW_DOWN_START, TURN_FINE_TUNE_START, TURN_POWER_MIN, PIDController.TypePID.DRIVE, motorTopRight, motorTopLeft, motorBottomLeft, motorBottomRight);
    }

    @Override
    public void loop() {
        switch(state) {
            case PUSHING_BUTTON:
                pushButtonSequence();
                break;
            case DROPPING_CLIMBERS:
                dropClimbers();
                break;
            case PARKING:
                park();
                break;
        }
    }

    private void pushButtonSequence() {
        switch(pushButtonState) {
            case TURNING:
                turnToButton();
            case PUSHING_BUTTON:
                pressButton();
        }
    }

    private void turnToButton() {
        driveFoward();
    }

    private void pressButton() {
        doForTime(2000, new Action() {
            @Override
            public void action() {
                buttonPusher.setPosition(1);
            }

            @Override
            public void onComplete() {
                buttonPusher.setPosition(0);
                state = State.DROPPING_CLIMBERS;
            }
        });
    }

    private void dropClimbers() {

    }

    private void park() {

    }

    private void doForTime (int time, Action action) {
        if (actionStartTime == 0) {
            actionStartTime = System.currentTimeMillis();
            action.action();
        } else if (System.currentTimeMillis() - actionStartTime > time) {
            actionStartTime = 0;
            action.onComplete();
        }
    }

    private void driveForward(double distance) {
        drivePIDController.setTarget(distance);
        if (!drivePIDController.hasReachedDestination())
            setMotors(drivePIDController.run());
    }

    private void turn(double degrees) {
        drivePIDController.setTarget(degrees);
        if (!drivePIDController.hasReachedDestination())
            setMotors(drivePIDController.run());
    }

    private void setMotors(double[] power) {
        powerRight = power[0];
        powerLeft = power[1];
        if (powerLeft > powerRight)
            turnState = TurnState.TURNING_LEFT;
        else if (powerLeft < powerRight)
            turnState = TurnState.TURNING_RIGHT;
        else
            turnState = TurnState.FORWARD;

        motorTopRight.setPower(powerRight);
        motorBottomRight.setPower(powerRight);
        motorTopLeft.setPower(powerLeft);
        motorBottomLeft.setPower(powerLeft);
    }
}
