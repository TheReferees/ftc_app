package com.thereferees.ftc_app.OpModes;

import android.util.Log;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.thereferees.ftc_app.OpModes.lib.Action;
import com.thereferees.ftc_app.OpModes.lib.Callback;
import com.thereferees.ftc_app.OpModes.lib.Condition;
import com.thereferees.ftc_app.OpModes.lib.EncoderPIDController;
import com.thereferees.ftc_app.OpModes.lib.GyroPIDControllerNew;
import com.thereferees.ftc_app.OpModes.lib.PIDController;

import org.swerverobotics.library.ClassFactory;
import org.swerverobotics.library.SynchronousOpMode;
import org.swerverobotics.library.interfaces.Autonomous;
import org.swerverobotics.library.interfaces.EulerAngles;
import org.swerverobotics.library.interfaces.IBNO055IMU;
import org.swerverobotics.library.interfaces.IFunc;
import org.swerverobotics.library.interfaces.Position;

/**
 * A skeletal example of a do-nothing first OpMode. Go ahead and change this code
 * to suit your needs, or create sibling OpModes adjacent to this one in the same
 * Java package.
 */
@Autonomous(name="AutonRedNoDelayBack")
public class AutonRedNoDelayBack extends SynchronousOpMode {
    public enum State {
        PUSHING_BUTTON, PARKING
    }

    public enum PushButtonState {
        FORWARD1, TURN1, FORWARD2, TURN2, FORWARD3, DETECT_BUTTON, EXTEND_PUSHER, PUSH_BUTTON, RETRACT_PUSHER, FINISH, RETRACT_CLIMBER_DROP, CLIMBER_DROP
    }

    ColorSensor colorSensor;

    Servo buttonPusher;
    Servo climberDrop;

    IBNO055IMU gyro;
    IBNO055IMU.Parameters parameters;

    DcMotor motorTopRight;
    DcMotor motorTopLeft;
    DcMotor motorBottomRight;
    DcMotor motorBottomLeft;
    DcMotor motorPickup;

    EulerAngles angles;
    Position position;

    private double startAngle = 0;

    private long actionStartTime;
    private State state;
    private PushButtonState pushButtonState;

    private boolean actionBeenInitialized = false;

    private boolean isFirstExtend = true;

    final int       DRIVE_GEAR_RATIO        = 1,
                    DRIVE_THRESHOLD         = 60;

    final double    DRIVE_WHEEL_DIAMETER    = 4,
                    DRIVE_SLOW_DOWN_START   = 1.5d,
                    DRIVE_FINE_TUNE_START   = 0.5d,
                    GYRO_SLOW_DOWN_START    = 45d,
                    GYRO_FINE_TUNE_START    = 15d,
                    GYRO_THRESHOLD          = 1.5;

    final double PICKUP_POWER = 0.8d;

    private boolean pause = false;


    private EncoderPIDController drivePIDController;
    private GyroPIDControllerNew turnPIDController;

    public void initialize() {
        actionStartTime = 0;
        state = State.PUSHING_BUTTON;
        motorTopRight = hardwareMap.dcMotor.get("M_driveFR");
        motorTopLeft = hardwareMap.dcMotor.get("M_driveFL");
        motorBottomRight = hardwareMap.dcMotor.get("M_driveBR");
        motorBottomLeft = hardwareMap.dcMotor.get("M_driveBL");
        motorPickup    = hardwareMap.dcMotor.get("M_pickup");
        Log.d("Init...", "HERE");

        //colorSensor = hardwareMap.colorSensor.get("SS_color");
        //buttonPusher = hardwareMap.servo.get("S_buttonPusher");
        //climberDrop = hardwareMap.servo.get("S_climberDrop");

        motorTopRight.setDirection(DcMotor.Direction.REVERSE);
        motorBottomRight.setDirection(DcMotor.Direction.REVERSE);

        pushButtonState = PushButtonState.FORWARD1;

        Log.d("Init...", "HERE1");

        parameters = new IBNO055IMU.Parameters();
        parameters.angleUnit      = IBNO055IMU.ANGLEUNIT.DEGREES;
        parameters.accelUnit      = IBNO055IMU.ACCELUNIT.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        parameters.loggingTag     = "BNO055";
        gyro = ClassFactory.createAdaFruitBNO055IMU(hardwareMap.i2cDevice.get("SS_gyro"), parameters);

        Log.d("Init...", "HERE2");
        drivePIDController = new EncoderPIDController(DRIVE_WHEEL_DIAMETER, DRIVE_GEAR_RATIO, DRIVE_THRESHOLD, DRIVE_SLOW_DOWN_START, DRIVE_FINE_TUNE_START, PIDController.TypePID.DRIVE, motorTopRight, motorBottomRight, motorTopLeft, motorBottomLeft);
        turnPIDController = new GyroPIDControllerNew(gyro, GYRO_THRESHOLD, GYRO_SLOW_DOWN_START, GYRO_FINE_TUNE_START, motorTopRight, motorBottomRight, motorTopLeft, motorBottomLeft);
        startAngle = gyro.getAngularOrientation().heading;

        Log.d("InitAuton", "" + gyro.isGyroCalibrated());
    }

    @Override
    public void main() throws InterruptedException {
        initialize();
        //composeDashboard();
        while (opModeIsActive()) {
            if (!gyro.isGyroCalibrated())
                idle();
            iteration();
            telemetry.update();
            idle();
        }
    }

    public void iteration() {
        if (true) {
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
        /*telemetry.addData("Drive R Pos: ", drivePIDController.getCurrentPosition()[0]);
        telemetry.addData("Drive L Pos: ", drivePIDController.getCurrentPosition()[1]);
        telemetry.addData("Drive R Target: ", drivePIDController.getTargets()[0]);
        telemetry.addData("Drive L Target: ", drivePIDController.getTargets()[1]);
        telemetry.addData("DRIVE R POWER: ", drivePIDController.run()[0]);
        telemetry.addData("DRIVE L POWER: ", drivePIDController.run()[1]);*/

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
                driveForward(24d, 10000, new Callback() {
                    @Override
                    public void onComplete() {
                        Log.d("AutonForward", "setting enum");
                        pushButtonState = PushButtonState.TURN1;
                    }
                });
                break;
            case TURN1:
                Log.d("AutonTurn", "BEFORE TURNING");
                turn(-45, 10000, new Callback() {
                    @Override
                    public void onComplete() {
                        //state=State.PARKING;
                        pushButtonState = PushButtonState.FORWARD2;
                    }
                });
                break;
            case FORWARD2:
                driveForward(80d, 10000, new Callback() {
                    @Override
                    public void onComplete() {
                        //state=State.PARKING;
                        pushButtonState = PushButtonState.TURN2;
                    }
                });
                break;
            case TURN2:
                turn(45, 10000, new Callback() {
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
                        pushButtonState = PushButtonState.CLIMBER_DROP;
                    }
                });
                break;
            case CLIMBER_DROP:
                climberDrop.setPosition(Servo.MIN_POSITION);
                /*if (climberDrop.getPosition() - Servo.MIN_POSITION > 0.05d)
                    climberDrop.setPosition(climberDrop.getPosition() - 0.01);*/
                if (climberDrop.getPosition() - Servo.MIN_POSITION < 0.05d) {
                    climberDrop.setPosition(0.36d);
                    state = State.PARKING;
                }
                break;
            /*case EXTEND_PUSHER:
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

                break;*/
            /*case PUSHING_BUTTON:
                pressButton();*/
        }
    }

    private void dropClimbers() {

        state = State.PARKING;
    }

    private void park() {
        driveForward(-12d, 10000, new Callback() {
            @Override
            public void onComplete() {
                stopMotors();
            }
        });
    }

    private void waitForConfirmation() {
        pause = true;
        motorPickup.setPower(0);
        //telemetry.addData("pause", "PAUSING");
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
                Log.d("InitAuton", "" + startAngle);
                drivePIDController.setTargets(distance);
            }

            @Override
            public void action() {
                setMotors(drivePIDController.run());
            }

            @Override
            public void onComplete() {
                Log.d("AutonForward", "HEREE");
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
                Log.d("TURNING", "RIGHT: " + turnPIDController.run()[0]);
                Log.d("TURNING", "LEFT: " + turnPIDController.run()[1]);
                setMotors(turnPIDController.run());
            }

            @Override
            public void onComplete() {
                Log.d("AutonTurn1", "COMPLETE");
                Log.d("InitAuton", "" + gyro.getAngularOrientation().heading);
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













    void composeDashboard()
    {
        // The default dashboard update rate is a little too slow for our taste here, so we update faster
        telemetry.setUpdateIntervalMs(200);

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() {
            @Override
            public void run() {
                // Acquiring the angles is relatively expensive; we don't want
                // to do that in each of the three items that need that info, as that's
                // three times the necessary expense.
                angles = gyro.getAngularOrientation();
                position = gyro.getPosition();
            }
        });
        telemetry.addLine(
                telemetry.item("status: ", new IFunc<Object>() {
                    public Object value() {
                        return decodeStatus(gyro.getSystemStatus());
                    }
                }),
                telemetry.item("calib: ", new IFunc<Object>() {
                    public Object value() {
                        return decodeCalibration(gyro.read8(IBNO055IMU.REGISTER.CALIB_STAT));
                    }
                }));

        telemetry.addLine(
                telemetry.item("heading: ", new IFunc<Object>() {
                    public Object value() {
                        return formatAngle(angles.heading);
                    }
                }),
                telemetry.item("pitch: ", new IFunc<Object>() {
                    public Object value() {
                        return formatAngle(angles.pitch);
                    }
                }));

        telemetry.addLine(
                telemetry.item("x: ", new IFunc<Object>() {
                    public Object value() {
                        return formatPosition(position.x);
                    }
                }),
                telemetry.item("y: ", new IFunc<Object>() {
                    public Object value() {
                        return formatPosition(position.y);
                    }
                }),
                telemetry.item("z: ", new IFunc<Object>() {
                    public Object value() {
                        return formatPosition(position.z);
                    }
                }));
    }
    String decodeStatus(int status)
    {
        switch (status)
        {
            case 0: return "idle";
            case 1: return "syserr";
            case 2: return "periph";
            case 3: return "sysinit";
            case 4: return "selftest";
            case 5: return "fusion";
            case 6: return "running";
        }
        return "unk";
    }

    /** Turn a calibration code into something that is reasonable to show in telemetry */
    String decodeCalibration(int status)
    {
        StringBuilder result = new StringBuilder();

        result.append(String.format("s%d", (status >> 2) & 0x03));  // SYS calibration status
        result.append(" ");
        result.append(String.format("g%d", (status >> 2) & 0x03));  // GYR calibration status
        result.append(" ");
        result.append(String.format("a%d", (status >> 2) & 0x03));  // ACC calibration status
        result.append(" ");
        result.append(String.format("m%d", (status >> 0) & 0x03));  // MAG calibration status

        return result.toString();
    }

    String formatAngle(double angle)
    {
        return parameters.angleUnit ==IBNO055IMU.ANGLEUNIT.DEGREES ? formatDegrees(angle) : formatRadians(angle);
    }

    String formatDegrees(double degrees)
    {
        return String.format("%.1f", normalizeDegrees(degrees));
    }
    String formatRate(double cyclesPerSecond)
    {
        return String.format("%.2f", cyclesPerSecond);
    }
    double normalizeDegrees(double degrees)
    {
        while (degrees >= 180.0) degrees -= 360.0;
        while (degrees < -180.0) degrees += 360.0;
        return degrees;
    }
    String formatRadians(double radians)
    {
        return formatDegrees(degreesFromRadians(radians));
    }
    double degreesFromRadians(double radians)
    {
        return radians * 180.0 / Math.PI;
    }
    String formatPosition(double coordinate)
    {
        String unit = parameters.accelUnit == IBNO055IMU.ACCELUNIT.METERS_PERSEC_PERSEC
                ? "m" : "??";
        return String.format("%.2f%s", coordinate, unit);
    }
}
