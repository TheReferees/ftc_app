package com.thereferees.ftc_app.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.thereferees.ftc_app.OpModes.lib.Action;
import com.thereferees.ftc_app.OpModes.lib.Callback;
import com.thereferees.ftc_app.OpModes.lib.Condition;
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

    public enum PushButtonState {
        MOVING, TURNING, PUSHING_BUTTON
    }

    DcMotor motorTopRight;
    DcMotor motorTopLeft;
    DcMotor motorBottomRight;
    DcMotor motorBottomLeft;
    DcMotor motorPickup;

    //Servo buttonPusher;

    private long actionStartTime;
    private State state;
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

    final double PICKUP_POWER = 0.8d;


    private PIDController drivePIDController;
    private PIDController turnPIDController;

    @Override
    public void init() {
        actionStartTime = 0;
        state = State.PUSHING_BUTTON;
        motorTopRight = hardwareMap.dcMotor.get("M_driveFR");
        motorTopLeft = hardwareMap.dcMotor.get("M_driveFL");
        motorBottomRight = hardwareMap.dcMotor.get("M_driveBR");
        motorBottomLeft = hardwareMap.dcMotor.get("M_driveBL");
        motorPickup    = hardwareMap.dcMotor.get("M_pickup");

        motorTopRight.setDirection(DcMotor.Direction.REVERSE);
        motorBottomRight.setDirection(DcMotor.Direction.REVERSE);

        //buttonPusher = hardwareMap.servo.get("buttonPusher");
        pushButtonState = PushButtonState.TURNING;

        drivePIDController = new PIDController(DRIVE_WHEEL_DIAMETER, DRIVE_GEAR_RATIO, DRIVE_THRESHOLD, DRIVE_SLOW_DOWN_START, DRIVE_FINE_TUNE_START, DRIVE_POWER_MIN, PIDController.TypePID.DRIVE, motorTopRight, motorTopLeft, motorBottomLeft, motorBottomRight);
        turnPIDController = new PIDController(DRIVE_WHEEL_DIAMETER, TURN_DIAMETER, DRIVE_GEAR_RATIO, TURN_THRESHOLD, TURN_SLOW_DOWN_START, TURN_FINE_TUNE_START, TURN_POWER_MIN, PIDController.TypePID.TURN, motorTopRight, motorTopLeft, motorBottomLeft, motorBottomRight);
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
            case MOVING:
                motorPickup.setPower(-PICKUP_POWER);
                driveForward(36d, new Callback() {
                    @Override
                    public void onComplete() {
                        pushButtonState = PushButtonState.TURNING;
                    }
                });
            case TURNING:
                turn(120, new Callback() {
                    @Override
                    public void onComplete() {
                        state = State.DROPPING_CLIMBERS;
                    }
                });
            /*case PUSHING_BUTTON:
                pressButton();*/
        }
    }

    /*private void pressButton() {
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
    }*/

    private void dropClimbers() {
        state = State.PARKING;
    }

    private void park() {
        stopMotors();
    }

    private void doUntil(Condition condition, Action action) {
        if (!action.hasInitialized) {
            action.init();
            action.hasInitialized = true;
        }

        if (!condition.evaluate()) {
            action.action();
        } else {
            action.onComplete();
        }
    }

    private void doForTime(int time, Action action) {
        if (actionStartTime == 0) {
            actionStartTime = System.currentTimeMillis();
            action.action();
        } else if (System.currentTimeMillis() - actionStartTime > time) {
            actionStartTime = 0;
            action.onComplete();
        }
    }

    private void driveForward(final double distance, final Callback callback) {
        doUntil(new Condition() {
            @Override
            public boolean evaluate() {
                return drivePIDController.hasReachedDestination();
            }
        }, new Action() {
            @Override
            public void init() {
                drivePIDController.setTargets(distance);
            }

            @Override
            public void action() {
                setMotors(drivePIDController.run());
            }

            @Override
            public void onComplete() {
                callback.onComplete();
            }
        });
    }

    private void turn(final double degrees, final Callback callback) {
        doUntil(new Condition() {
            @Override
            public boolean evaluate() {
                return turnPIDController.hasReachedDestination();
            }
        }, new Action() {
            @Override
            public void init() {
                turnPIDController.setTargets(degrees);
            }

            @Override
            public void action() {
                setMotors(turnPIDController.run());
            }

            @Override
            public void onComplete() {
                callback.onComplete();
            }
        });
    }

    private void setMotors(double[] power) {
        double powerRight = power[0];
        double powerLeft = power[1];

        motorTopRight.setPower(powerRight);
        motorBottomRight.setPower(powerRight);
        motorTopLeft.setPower(powerLeft);
        motorBottomLeft.setPower(powerLeft);
    }

    private void stopMotors() {
        setMotors(new double[2]);
        motorPickup.setPower(0);
    }
}
