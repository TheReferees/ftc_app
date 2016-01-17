package com.thereferees.ftc_app.OpModes;

import com.thereferees.ftc_app.utilities.Action;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

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

    @Override
    public void init() {
        actionStartTime = 0;
        state = State.PUSHING_BUTTON;
        motorTopRight = hardwareMap.dcMotor.get("topRight");
        motorTopLeft = hardwareMap.dcMotor.get("topLeft");
        motorBottomRight = hardwareMap.dcMotor.get("bottomRight");
        motorBottomLeft = hardwareMap.dcMotor.get("bottomLeft");
        buttonPusher = hardwareMap.servo.get("buttonPusher");
        turnState = TurnState.FORWARD;
        pushButtonState = PushButtonState.TURNING;
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
        doForTime(5000, new Action() {
            @Override
            public void action() {
                setMotorsTurnLeft();
            }

            @Override
            public void onComplete() {
                pushButtonState = PushButtonState.PUSHING_BUTTON;
            }
        });
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

    private void setMotorsTurnRight() {
        setMotors(0.5, 1);
    }

    private void setMotorsTurnLeft() {
        setMotors(1, 0.5);
    }

    private void setMotorsForward() {
        setMotors(1, 1);
    }

    private void setMotorsBackward() {
        setMotors(-1, -1);
    }

    private void setMotors(double powerLeft, double powerRight) {
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
