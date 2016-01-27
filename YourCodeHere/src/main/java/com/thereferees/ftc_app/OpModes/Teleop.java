package com.thereferees.ftc_app.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.util.Range;

import org.swerverobotics.library.interfaces.TeleOp;

/**
 * Created by Saba on 1/26/16.
 */
@TeleOp(name="TeleOp")
public class Teleop extends OpMode {

    // motor declarations
    DcMotor M_driveBR   = null,
            M_driveBL   = null,
            M_driveFR   = null,
            M_driveFL   = null,
            M_liftR     = null,
            M_liftL     = null,
            M_pickup    = null,
            M_basket    = null;

    // servo declarations
    Servo   S_basket              = null,
            //S_buttonPusher        = null,
            S_climberDrop         = null,
            S_climberKnockdownR   = null,
            S_climberKnockdownL   = null;
    //S_colorSensorGround   = null;

    final double C_STICK_TOP_THRESHOLD = 0.90d;

    // constant powers
    final double    STOP            = 0.0d,
                    DRIVE_POWER     = 0.8d,
                    TURN_POWER      = 0.8d,
                    PICKUP_POWER    = 0.8d;

    // starting servo positions
    final double    S_BASKET_START_POS      = Servo.MIN_POSITION,
                    S_CLIMBER_DROP_START_POS        = 0.0d,
                    S_CLIMBER_KNOCKDOWN_R_START_POS = Servo.MAX_POSITION,
                    S_CLIMBER_KNOCKDOWN_L_START_POS = Servo.MIN_POSITION;

    // ending servo positions
    final double    S_BASKET_END_POS              = Servo.MAX_POSITION,
                    S_CLIMBER_DROP_END_POS        = 0.0d,
                    S_CLIMBER_KNOCKDOWN_R_END_POS = 0.37d,
                    S_CLIMBER_KNOCKDOWN_L_END_POS = 0.54d;

    double          S_climberKnockdownRTargetEndPos = S_CLIMBER_KNOCKDOWN_R_END_POS,
                    S_climberKnockdownLTargetEndPos = S_CLIMBER_KNOCKDOWN_L_END_POS;

    final double TURN_THRESHOLD     = 3.0d;

    // drive powers
    double          M_drivePowerR           = STOP,
            M_drivePowerL           = STOP,
            M_liftPowerR            = STOP,
            M_liftPowerL            = STOP,
            M_pickupPower           = STOP,
            M_basketPower           = STOP;

    // servo positions
    double  S_basketPosition         = S_BASKET_START_POS,
            S_climberDropPos        = S_CLIMBER_DROP_START_POS,
            S_climberKnockdownRPos  = S_CLIMBER_KNOCKDOWN_R_START_POS,
            S_climberKnockdownLPos  = S_CLIMBER_KNOCKDOWN_L_START_POS;

    private double convertStick(float controllerValue) {   return Math.sin(Range.clip(controllerValue * Math.PI / 2 / C_STICK_TOP_THRESHOLD, -Math.PI / 2, Math.PI / 2)); }
    private Quadrant findQuadrant(Gamepad gamepad, String stick) {
        if(stick.toLowerCase().equals("left")) {
            if (gamepad.left_stick_y != 0.0d && gamepad.left_stick_x != 0.0d) {
                if (-gamepad.left_stick_y >= 0.0d) {
                    if (gamepad.left_stick_x >= 0.0d) {
                        return Quadrant.Q1;
                    } else {
                        return Quadrant.Q2;
                    }
                } else {
                    if (gamepad.left_stick_x >= 0.0d) {
                        return Quadrant.Q3;
                    } else {
                        return Quadrant.Q4;
                    }
                }
            } else {
                return Quadrant.NA;
            }
        } else {
            if (gamepad.right_stick_y != 0.0d && gamepad.right_stick_x != 0.0d) {
                if (-gamepad.right_stick_y >= 0.0d) {
                    if (gamepad.right_stick_x >= 0.0d) {
                        return Quadrant.Q1;
                    } else {
                        return Quadrant.Q2;
                    }
                } else {
                    if (gamepad.right_stick_x >= 0.0d) {
                        return Quadrant.Q3;
                    } else {
                        return Quadrant.Q4;
                    }
                }
            } else {
                return Quadrant.NA;
            }
        }
    }
    private double findAngle() {
        final double TICKS_TO_DEGREES = 0.0d;
        return (((M_driveFR.getCurrentPosition() + M_driveBR.getCurrentPosition()) / 2.0d) - ((M_driveFL.getCurrentPosition() + M_driveBL.getCurrentPosition()) / 2.0d)) * TICKS_TO_DEGREES % 360.0d;
    }
    private void turn(double angle) {
        DcMotor[] motors = {M_driveFR, M_driveFL, M_driveBR, M_driveBR};
        double kP = 0.000d;
        double kI;
        double accumError = 0.0d;
        double thresholdPower = 0.1d;
        double error = findAngle() - angle;
        accumError += error;
        double PIDValue = error * kP;
        if(Math.abs(error) <= TURN_THRESHOLD) {
            M_drivePowerR = STOP;
            M_drivePowerL = STOP;
        } else {
            PIDValue = Range.clip(PIDValue, -1.0d, 1.0d);
            M_drivePowerR = PIDValue;
            M_drivePowerL = -PIDValue;
        }
    }

    double targetAngleL, targetAngleR;
    enum DriveMode{
        TANK,
        BUTTON,
        THRUST,
        ARCADE
    } DriveMode driveMode = DriveMode.TANK;
    enum Quadrant {
        Q1,
        Q2,
        Q3,
        Q4,
        NA
    }
    enum ServoMode {
        AIM,
        HIT
    } ServoMode servoMode = ServoMode.AIM;
    enum ServoPos {
        OUT,
        IN
    }   ServoPos servoPosR = ServoPos.IN;
    ServoPos servoPosL = ServoPos.IN;

    void grabMotors() {
        M_driveFR   = hardwareMap.dcMotor.get("M_driveFR");
        M_driveFL   = hardwareMap.dcMotor.get("M_driveFL");
        M_driveBR   = hardwareMap.dcMotor.get("M_driveBR");
        M_driveBL   = hardwareMap.dcMotor.get("M_driveBL");
        M_liftR     = hardwareMap.dcMotor.get("M_liftR");
        M_liftL     = hardwareMap.dcMotor.get("M_liftL");
        M_pickup    = hardwareMap.dcMotor.get("M_pickup");
        M_basket    = hardwareMap.dcMotor.get("M_basket");
    }
    void configureMotors() {
        M_driveFR.setDirection(DcMotor.Direction.REVERSE);
        M_driveBR.setDirection(DcMotor.Direction.REVERSE);
    }
    void grabServos() {
        S_basket = hardwareMap.servo.get("S_basket");
        S_climberDrop = hardwareMap.servo.get("S_climberDrop");
        S_climberKnockdownR = hardwareMap.servo.get("S_climberKnockdownR");
        S_climberKnockdownL = hardwareMap.servo.get("S_climberKnockdownL");
    }

    void grabSensors() {

    }

    @Override
    public void init() {
        grabMotors();
        configureMotors();
        grabServos();
        grabSensors();
    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {
        /*
        if(gamepad1.dpad_up) {
            driveMode = DriveMode.TANK;
        } else if(gamepad1.dpad_left) {
            driveMode = DriveMode.BUTTON;
        } else if(gamepad1.dpad_down) {
            driveMode = DriveMode.THRUST;
        } else if(gamepad1.dpad_right) {
            driveMode = DriveMode.ARCADE;
        }

        switch (driveMode) {
            case TANK:
                M_drivePowerR = convertStick(-gamepad1.right_stick_y);
                M_drivePowerL = convertStick(-gamepad1.left_stick_y);
                telemetry.addData("Drive Mode: ", "Tank");
                break;
            case BUTTON:
                // drive forward
                if(gamepad1.y) {
                    M_drivePowerR = DRIVE_POWER;
                    M_drivePowerL = DRIVE_POWER;
                // drive backward
                } else if(gamepad1.a) {
                    M_drivePowerR = -DRIVE_POWER;
                    M_drivePowerL = -DRIVE_POWER;
                // turn to the right
                } else if(gamepad1.b) {
                    M_drivePowerR = -TURN_POWER;
                    M_drivePowerL = TURN_POWER;
                // turn to the left
                } else if(gamepad1.x) {
                    M_drivePowerR = TURN_POWER;
                    M_drivePowerL = -TURN_POWER;
                }
                telemetry.addData("Drive Mode: ", "Button");
                break;
            case THRUST:
                Quadrant quadrant = findQuadrant(gamepad1, "left");
                switch (quadrant) {
                    case Q1:
                    case Q4:
                        targetAngleL = 90.0d - Math.toDegrees(Math.asin(-gamepad1.left_stick_y / gamepad1.left_stick_x));
                        break;
                    case Q2:
                    case Q3:
                        targetAngleL = -90.0d - Math.toDegrees(-Math.asin(-gamepad1.left_stick_y / gamepad1.left_stick_x));
                        break;
                    default:
                        targetAngleL = findAngle();
                        break;
                }
                if(Math.abs(targetAngleL - findAngle()) > TURN_THRESHOLD) {
                    turn(targetAngleL);
                } else {
                    M_drivePowerR = convertStick(-gamepad1.right_stick_y);
                    M_drivePowerL = convertStick(-gamepad1.right_stick_y);
                }
                telemetry.addData("Drive Mode: ", "Thrust");
                break;
            case ARCADE:
                Quadrant quadrantL = findQuadrant(gamepad1, "left");
                switch (quadrantL) {
                    case Q1:
                    case Q4:
                        targetAngleL = 90.0d - Math.toDegrees(Math.asin(-gamepad1.left_stick_y / gamepad1.left_stick_x));
                        break;
                    case Q2:
                    case Q3:
                        targetAngleL = -90.0d - Math.toDegrees(-Math.asin(-gamepad1.left_stick_y / gamepad1.left_stick_x));
                        break;
                    default:
                        targetAngleL = findAngle();
                        break;
                }
                if(Math.abs(targetAngleL - findAngle()) > TURN_THRESHOLD) {
                    turn(targetAngleL);
                } else {
                    M_drivePowerR = convertStick(-gamepad1.left_stick_y / gamepad1.left_stick_x);
                    M_drivePowerL = convertStick(-gamepad1.left_stick_y / gamepad1.left_stick_x);
                }
                if(M_drivePowerR == 0.0d && M_drivePowerL == 0.0d) {
                    Quadrant quadrantR = findQuadrant(gamepad1, "right");
                    switch (quadrantR) {
                        case Q1:
                        case Q4:
                            targetAngleR = 90.0d - Math.toDegrees(Math.asin(-gamepad1.left_stick_y / gamepad1.left_stick_x));
                            break;
                        case Q2:
                        case Q3:
                            targetAngleR = -90.0d - Math.toDegrees(-Math.asin(-gamepad1.left_stick_y / gamepad1.left_stick_x));
                            break;
                        default:
                            targetAngleR = findAngle();
                            break;
                    }
                    turn(targetAngleR);
                }
                telemetry.addData("Drive Mode: ", "Arcade");
                break;
            default:
                break;
        }*/

        // drive base control block
        M_drivePowerR = convertStick(-gamepad1.right_stick_y);
        M_drivePowerL = convertStick(-gamepad1.left_stick_y);
        // pickup control block
        if(gamepad1.right_bumper) {
            M_pickupPower = PICKUP_POWER;
        } else if(gamepad1.left_bumper) {
            M_pickupPower = -PICKUP_POWER;
        } else {
            M_pickupPower = STOP;
        }
        // basket control block
        if(gamepad1.right_trigger > 0.0d) {
            M_basketPower = gamepad1.right_trigger;
        } else if(gamepad1.left_trigger > 0.0d) {
            M_basketPower = -gamepad1.left_trigger;
        } else {
            M_basketPower = STOP;
        }

        // lift control block
        M_liftPowerR = convertStick(-gamepad2.right_stick_y);
        M_liftPowerL = convertStick(-gamepad2.left_stick_y);

        if(gamepad1.dpad_down) {
            servoMode = ServoMode.AIM;
            S_climberKnockdownLTargetEndPos = S_CLIMBER_KNOCKDOWN_L_END_POS;
            S_climberKnockdownRTargetEndPos = S_CLIMBER_KNOCKDOWN_R_END_POS;
        } else if(gamepad1.dpad_up) {
            servoMode = ServoMode.HIT;
            S_climberKnockdownLTargetEndPos = Servo.MAX_POSITION;
            S_climberKnockdownRTargetEndPos = Servo.MIN_POSITION;
        }
        if(gamepad1.dpad_left) {
            switch (servoPosL) {
                case IN:
                    S_climberKnockdownLPos = S_climberKnockdownLTargetEndPos;
                    servoPosL = ServoPos.OUT;
                    break;
                case OUT:
                    S_climberKnockdownLPos = S_CLIMBER_KNOCKDOWN_L_START_POS;
                    servoPosL = ServoPos.IN;
                    break;
                default:
                    break;
            }
        } else if(gamepad1.dpad_right) {
            switch (servoPosR) {
                case IN:
                    S_climberKnockdownRPos = S_climberKnockdownRTargetEndPos;
                    servoPosR = ServoPos.OUT;
                    break;
                case OUT:
                    S_climberKnockdownRPos = S_CLIMBER_KNOCKDOWN_R_START_POS;
                    servoPosR = ServoPos.IN;
                    break;
                default:
                    break;
            }
        }

        if(gamepad1.a) {
            S_basketPosition = S_BASKET_END_POS;
        } else if(gamepad1.y) {
            S_basketPosition = S_BASKET_START_POS;
        }

        M_driveFR.setPower(M_drivePowerR);
        M_driveFL.setPower(M_drivePowerL);
        M_driveBR.setPower(M_drivePowerR);
        M_driveBL.setPower(M_drivePowerL);
        M_liftR.setPower(M_liftPowerR);
        M_liftL.setPower(M_liftPowerL);
        M_pickup.setPower(M_pickupPower);
        M_basket.setPower(M_basketPower);

        S_climberDrop.setPosition(S_climberDropPos);
        S_climberKnockdownR.setPosition(S_climberKnockdownRPos);
        S_climberKnockdownL.setPosition(S_climberKnockdownLPos);

        telemetry.addData("Text", "*** Robot Data***");
        telemetry.addData("Climber Drop Pos", S_climberDropPos);
        telemetry.addData("ClimberKnockdownR Pos", S_climberKnockdownRPos);
        telemetry.addData("ClimberKnockdownL Pos", S_climberKnockdownLPos);
        telemetry.addData("RF POS", M_driveFR.getCurrentPosition());
        telemetry.addData("LF POS", M_driveFL.getCurrentPosition());
        telemetry.addData("RB POS", M_driveBR.getCurrentPosition());
        telemetry.addData("LB POS", M_driveBL.getCurrentPosition());
    }

    @Override
    public void stop() {
        M_driveFR.setPower(STOP);
        M_driveFL.setPower(STOP);
        M_driveBR.setPower(STOP);
        M_driveBL.setPower(STOP);
        M_liftR.setPower(STOP);
        M_liftL.setPower(STOP);
        M_pickup.setPower(STOP);
        M_basket.setPower(STOP);
    }
}

