package com.thereferees.ftc_app.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
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
        PUSHING_BUTTON, PARKING
    }

    public enum PushButtonState {
        FORWARD1, TURN1, FORWARD2, TURN2, FORWARD3, DETECT_BUTTON, EXTEND_PUSHER, PUSH_BUTTON, RETRACT_PUSHER, FINISH, RETRACT_CLIMBER_DROP
    }

    ColorSensor colorSensor;

    Servo buttonPusher;
    Servo climberDrop;

    DcMotor motorTopRight;
    DcMotor motorTopLeft;
    DcMotor motorBottomRight;
    DcMotor motorBottomLeft;
    DcMotor motorPickup;

    private long actionStartTime;
    private State state;
    private PushButtonState pushButtonState;

    private boolean actionBeenInitialized = false;

    private boolean isFirstExtend = true;

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
                    TURN_DIAMETER           = Math.sqrt(Math.pow(5.0 + 7.0/8, 2.0d) + Math.pow(16.0 + 3.0/8, 2.0d));

    final double PICKUP_POWER = 0.8d;

    private boolean pause = false;


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

        //colorSensor = hardwareMap.colorSensor.get("SS_color");
        //buttonPusher = hardwareMap.servo.get("S_buttonPusher");

        //climberDrop = hardwareMap.servo.get("S_climberDrop");

        motorTopRight.setDirection(DcMotor.Direction.REVERSE);
        motorBottomRight.setDirection(DcMotor.Direction.REVERSE);

        //buttonPusher = hardwareMap.servo.get("buttonPusher");
        pushButtonState = PushButtonState.FORWARD1;

        drivePIDController = new PIDController(DRIVE_WHEEL_DIAMETER, DRIVE_GEAR_RATIO, DRIVE_THRESHOLD, DRIVE_SLOW_DOWN_START, DRIVE_FINE_TUNE_START, DRIVE_POWER_MIN, PIDController.TypePID.DRIVE, motorTopRight, motorBottomRight, motorTopLeft, motorBottomLeft);
        turnPIDController = new PIDController(DRIVE_WHEEL_DIAMETER, TURN_DIAMETER, DRIVE_GEAR_RATIO, TURN_THRESHOLD, TURN_SLOW_DOWN_START, TURN_FINE_TUNE_START, TURN_POWER_MIN, PIDController.TypePID.TURN, motorTopRight, motorBottomRight, motorTopLeft, motorBottomLeft);
    }

    @Override
    public void loop() {
        if (!pause) {
            switch (state) {
                case PUSHING_BUTTON:
                    pushButtonSequence();
                    break;
                case PARKING:
                    park();
                    break;
            }
        } else if (gamepad1.a) {
            pause = false;
            motorPickup.setPower(-PICKUP_POWER);
        }
        telemetry.addData("Drive R Pos: ", drivePIDController.getCurrentPosition()[0]);
        telemetry.addData("Drive L Pos: ", drivePIDController.getCurrentPosition()[1]);
        telemetry.addData("Drive R Target: ", drivePIDController.getTargets()[0]);
        telemetry.addData("Drive L Target: ", drivePIDController.getTargets()[1]);
        telemetry.addData("DRIVE R POWER: ", drivePIDController.run()[0]);
        telemetry.addData("DRIVE L POWER: ", drivePIDController.run()[1]);

        /*telemetry.addData("Turn R Pos: ", turnPIDController.getCurrentPosition()[0]);
        telemetry.addData("Turn L Pos: ", turnPIDController.getCurrentPosition()[1]);
        telemetry.addData("Turn R Target: ", turnPIDController.getTargets()[0]);
        telemetry.addData("Turn L Target: ", turnPIDController.getTargets()[1]);
        telemetry.addData("Turn R POWER: ", turnPIDController.run()[0]);
        telemetry.addData("Turn L POWER: ", turnPIDController.run()[1]);*/
    }

    private void pushButtonSequence() {
        switch(pushButtonState) {
            case FORWARD1:
                motorPickup.setPower(-PICKUP_POWER);
                driveForward(35d, 10000, new Callback() {
                    @Override
                    public void onComplete() {
                        pushButtonState = PushButtonState.TURN1;
                    }
                });
                break;
            case TURN1:
                turn(180, 10000, new Callback() {
                    @Override
                    public void onComplete() {
                        pushButtonState = PushButtonState.FORWARD2;
                    }
                });
                break;
            case FORWARD2:
                driveForward(-24d, 10000, new Callback() {
                    @Override
                    public void onComplete() {
                        pushButtonState = PushButtonState.TURN2;
                    }
                });
                break;
            case TURN2:
                turn(-45, 10000, new Callback() {
                    @Override
                    public void onComplete() {
                        pushButtonState = PushButtonState.FORWARD3;
                    }
                });
                break;
            case FORWARD3:
                driveForward(15d, 10000, new Callback() {
                    @Override
                    public void onComplete() {
                        pushButtonState = PushButtonState.DETECT_BUTTON;
                    }
                });
                break;
            case DETECT_BUTTON:
                driveForward(5d, 10000, new Callback() {
                    @Override
                    public void onComplete() {
                        pushButtonState = PushButtonState.EXTEND_PUSHER;
                    }
                });
                break;
            case EXTEND_PUSHER:
                doUntil(5000, new Condition() {
                    @Override
                    public boolean evaluate() {
                        return colorSensor.red() > 0 || colorSensor.blue() > 0;
                    }
                }, new Action() {
                    @Override
                    public void init() {
                        buttonPusher.setPosition(1);
                    }

                    @Override
                    public void onComplete() {
                        buttonPusher.setPosition(0.5);

                        if (isFirstExtend) {
                            climberDrop.setPosition(1);

                            if (colorSensor.red() < colorSensor.blue()) {
                                pushButtonState = PushButtonState.RETRACT_PUSHER;
                                isFirstExtend = false;
                            } else {
                                pushButtonState = PushButtonState.PUSH_BUTTON;
                            }
                        } else {
                            pushButtonState = PushButtonState.PUSH_BUTTON;
                        }
                    }
                });
            case PUSH_BUTTON:
                doForTime(1000, new Action() {
                    @Override
                    public void init() {
                        buttonPusher.setPosition(1);
                    }

                    @Override
                    public void onComplete() {
                        buttonPusher.setPosition(0.5);
                        pushButtonState = PushButtonState.FINISH;
                    }
                });
                break;
            case RETRACT_PUSHER:
                doForTime(2000, new Action() {
                    @Override
                    public void init() {
                        buttonPusher.setPosition(0);
                    }

                    @Override
                    public void onComplete() {
                        buttonPusher.setPosition(0.5);
                        pushButtonState = PushButtonState.DETECT_BUTTON;
                    }
                });
                break;
            case FINISH:
                doForTime(2000, new Action() {
                    @Override
                    public void init() {
                        buttonPusher.setPosition(0);
                    }

                    @Override
                    public void onComplete() {
                        buttonPusher.setPosition(0.5);
                        pushButtonState = PushButtonState.RETRACT_CLIMBER_DROP;
                    }
                });
                break;
            case RETRACT_CLIMBER_DROP:
                doForTime(1000, new Action() {
                    @Override
                    public void init() {
                        climberDrop.setPosition(0);
                    }
                    @Override
                    public void onComplete() {
                        climberDrop.setPosition(0.5);
                        state = State.PARKING;
                    }
                });

                break;
            /*case PUSHING_BUTTON:
                pressButton();*/
        }
    }

    private void dropClimbers() {

        state = State.PARKING;
    }

    private void park() {
        stopMotors();
    }

    private void waitForConfirmation() {
        pause = true;
        motorPickup.setPower(0);
        telemetry.addData("pause", "PAUSING");
    }

    private void doUntil(int timeLimit, Condition condition, Action action) {
        if (!actionBeenInitialized) {
            action.init();
            actionBeenInitialized = true;
            Action.startTime = System.currentTimeMillis();
        }

        if (!condition.evaluate() && System.currentTimeMillis() - timeLimit <= Action.startTime) {
            action.action();
        } else {
            action.onComplete();
            actionBeenInitialized = false;
            waitForConfirmation();
        }
    }

    private void doForTime(int time, Action action) {
        doUntil(time, new Condition() {
            @Override
            public boolean evaluate() {
                return false;
            }
        }, action);
    }

    private void driveForward(final double distance, final int maxTime, final Callback callback) {
        doUntil(maxTime, new Condition() {
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

    private void turn(final double degrees, final int maxTime, final Callback callback) {
        doUntil(maxTime, new Condition() {
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
        motorPickup.setPower(0.0d);
    }
}
