package com.thereferees.ftc_app.OpModes.lib;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

// Do I want to have this class in a separate file or do I want to just put a private class in the teleop loop?

// stores all the controller variables and methods
public class Controller {

    // stores how far sticks are pushed
    volatile float  C1_stickRx, // controller 1's converted right stick value in the x direction
                    C1_stickRy, // controller 1's converted right stick value in the y direction
                    C1_stickLx, // controller 1's converted left stick value in the x direction
                    C1_stickLy, // controller 1's converted left stick value in the y direction
                    C2_stickRx, // controller 2's converted right stick value in the x direction
                    C2_stickRy, // controller 2's converted right stick value in the y direction
                    C2_stickLx, // controller 2's converted left stick value in the x direction
                    C2_stickLy; // controller 2's converted left stick value in the y direction

    // stores the angle sticks are pushed
    volatile double C1_AngleR, // controller 1's right stick angle
                    C1_AngleL, // controller 1's left stick angle
                    C2_AngleR, // controller 2's right stick angle
                    C2_AngleL; // controller 2's left stick angle

    // stores how far triggers are pressed
    volatile float  C1_triggerR, // controller 1's right trigger value
                    C1_triggerL, // controller 1's left trigger value
                    C2_triggerR, // controller 2's right trigger value
                    C2_triggerL; // controller 2's left trigger value

    // stores if triggers are pressed or not
    volatile boolean    isC1_triggerR = false,  // controller 1's right trigger pressed or not
                        isC1_triggerL = false,  // controller 1's left trigger pressed or not
                        isC2_triggerR = false,  // controller 2's right trigger pressed or not
                        isC2_triggerL = false;  // controller 2's left trigger pressed or not

    // stores if buttons are pressed or not
    volatile boolean    C1_A = false,       // controller 1's A button pressed or not
                        C1_B = false,       // controller 1's B button pressed or not
                        C1_X = false,       // controller 1's X button pressed or not
                        C1_Y = false,       // controller 1's Y button pressed or not
                        C1_Back = false,    // controller 1's Back button pressed or not
                        C1_Start = false,   // controller 1's Start button pressed or not
                        C2_A = false,       // controller 2's A button pressed or not
                        C2_B = false,       // controller 2's B button pressed or not
                        C2_X = false,       // controller 2's X button pressed or not
                        C2_Y = false,       // controller 2's Y button pressed or not
                        C2_Back = false,    // controller 2's Back button pressed or not
                        C2_Start = false;   // controller 2's Start button pressed or not

    // stores if d-pad buttons are pressed or not
    volatile boolean    C1_dUp = false,     // controller 1's d-pad up button pressed or not
                        C1_dRight = false,  // controller 1's d-pad right button pressed or not
                        C1_dDown = false,   // controller 1's d-pad down button pressed or not
                        C1_dLeft = false,   // controller 1's d-pad left button pressed or not
                        C2_dUp = false,     // controller 2's d-pad up button pressed or not
                        C2_dRight = false,  // controller 2's d-pad right button pressed or not
                        C2_dDown = false,   // controller 2's d-pad down button pressed or not
                        C2_dLeft = false;   // controller 2's d-pad left button pressed or not

    // controller thread object
    ControllerThread controllerThread;

    // constructor, takes in the two controllers
    public Controller(Gamepad pad1, Gamepad pad2) {
        // defines the controller thread object, passes in the two controllers
        controllerThread = new ControllerThread(pad1, pad1);
    }

    // updates all the controller values
    public void startThread() {
        controllerThread.run();
    }

    // controller thread
    private class ControllerThread extends Thread {

        // stores how far sticks are pushed
        volatile float  C1_stickRx, // controller 1's converted right stick value in the x direction
                        C1_stickRy, // controller 1's converted right stick value in the y direction
                        C1_stickLx, // controller 1's converted left stick value in the x direction
                        C1_stickLy, // controller 1's converted left stick value in the y direction
                        C2_stickRx, // controller 2's converted right stick value in the x direction
                        C2_stickRy, // controller 2's converted right stick value in the y direction
                        C2_stickLx, // controller 2's converted left stick value in the x direction
                        C2_stickLy; // controller 2's converted left stick value in the y direction

        // stores the angle sticks are pushed
        volatile double C1_AngleR, // controller 1's right stick angle
                        C1_AngleL, // controller 1's left stick angle
                        C2_AngleR, // controller 2's right stick angle
                        C2_AngleL; // controller 2's left stick angle

        // stores how far triggers are pressed
        volatile float  C1_triggerR, // controller 1's right trigger value
                        C1_triggerL, // controller 1's left trigger value
                        C2_triggerR, // controller 2's right trigger value
                        C2_triggerL; // controller 2's left trigger value

        // stores if triggers are pressed or not
        volatile boolean    isC1_triggerR = false,  // controller 1's right trigger pressed or not
                            isC1_triggerL = false,  // controller 1's left trigger pressed or not
                            isC2_triggerR = false,  // controller 2's right trigger pressed or not
                            isC2_triggerL = false;  // controller 2's left trigger pressed or not

        // stores if buttons are pressed or not
        volatile boolean    C1_A = false,       // controller 1's A button pressed or not
                            C1_B = false,       // controller 1's B button pressed or not
                            C1_X = false,       // controller 1's X button pressed or not
                            C1_Y = false,       // controller 1's Y button pressed or not
                            C1_Back = false,    // controller 1's Back button pressed or not
                            C1_Start = false,   // controller 1's Start button pressed or not
                            C2_A = false,       // controller 2's A button pressed or not
                            C2_B = false,       // controller 2's B button pressed or not
                            C2_X = false,       // controller 2's X button pressed or not
                            C2_Y = false,       // controller 2's Y button pressed or not
                            C2_Back = false,    // controller 2's Back button pressed or not
                            C2_Start = false;   // controller 2's Start button pressed or not

        // stores if d-pad buttons are pressed or not
        volatile boolean    C1_dUp = false,     // controller 1's d-pad up button pressed or not
                C1_dRight = false,  // controller 1's d-pad right button pressed or not
                C1_dDown = false,   // controller 1's d-pad down button pressed or not
                C1_dLeft = false,   // controller 1's d-pad left button pressed or not
                C2_dUp = false,     // controller 2's d-pad up button pressed or not
                C2_dRight = false,  // controller 2's d-pad right button pressed or not
                C2_dDown = false,   // controller 2's d-pad down button pressed or not
                C2_dLeft = false;   // controller 2's d-pad left button pressed or not

        // all of the not shared variables
        private final float C_STICK_TOP_THRESHOLD = 0.85f,      // least value for which stick value read from motor will be 1.0f
                            C_STICK_BOTTOM_THRESHOLD = 0.05f,   // greatest value for which stick value read from motor will be 0.0f
                            C_DPAD_BOTTOM_THRESHOLD = 0.1f;      // greatest value for which dpad will give no value

        // the two controllers
        private Gamepad pad1,
                pad2;

        // constructor, defines the two controlleres
        ControllerThread(Gamepad pad1, Gamepad pad2) {
            this.pad1 = pad1;
            this.pad2 = pad2;
            // set deadzones
            pad1.setJoystickDeadzone(C_STICK_BOTTOM_THRESHOLD);
            pad2.setJoystickDeadzone(C_STICK_BOTTOM_THRESHOLD);
        }

        // converts all of the controller sticks into more sensitive values
        // use a negative value for y axis since controller reads -1 when pushed forward
        private float convertStick(float controllerValue) {   return (float) Range.clip(Math.sin(controllerValue * Math.PI / 2 / C_STICK_TOP_THRESHOLD), -1.0d, 1.0d); }

        // the main loop function
        public void run() {
            try {
                while (!isInterrupted()) {

                    // finds stick values using the convert function
                    C1_stickRx = convertStick(pad1.right_stick_x);
                    C1_stickRy = convertStick(-pad1.right_stick_y);
                    C1_stickLx = convertStick(pad1.left_stick_x);
                    C1_stickLy = convertStick(-pad1.left_stick_y);
                    C2_stickRx = convertStick(pad2.right_stick_x);
                    C2_stickRy = convertStick(-pad2.right_stick_y);
                    C2_stickLx = convertStick(pad2.left_stick_x);
                    C2_stickLy = convertStick(-pad2.left_stick_y);

                    // finds stick angles
                    C1_AngleR = Math.toDegrees(Math.atan(pad1.right_stick_y/pad1.right_stick_x));
                    C1_AngleL = Math.toDegrees(Math.atan(pad1.left_stick_y/pad1.left_stick_x));
                    C2_AngleR = Math.toDegrees(Math.atan(pad2.right_stick_y/pad2.right_stick_x));
                    C2_AngleL = Math.toDegrees(Math.atan(pad2.left_stick_y/pad2.left_stick_x));

                    // finds how far triggers are pressed
                    C1_triggerR = pad1.right_trigger;
                    C1_triggerL = pad1.left_trigger;
                    C2_triggerR = pad2.right_trigger;
                    C2_triggerL = pad2.left_trigger;

                    // finds if triggers are pressed
                    isC1_triggerR = pad1.right_trigger > 0;
                    isC1_triggerL = pad1.left_trigger > 0;
                    isC2_triggerR = pad2.right_trigger > 0;
                    isC2_triggerL = pad2.left_trigger > 0;

                    // finds if buttons are pressed or not
                    C1_A = pad1.a;
                    C1_B = pad1.b;
                    C1_X = pad1.x;
                    C1_Y = pad1.y;
                    C1_Back = pad1.back;
                    C1_Start = pad1.start;
                    C2_A = pad2.a;
                    C2_B = pad2.b;
                    C2_X = pad2.x;
                    C2_Y = pad2.y;
                    C2_Back = pad2.back;
                    C2_Start = pad2.start;

                    // finds if d-pad buttons are pressed or not
                    C1_dUp = pad1.dpad_up;
                    C1_dRight = pad1.dpad_right;
                    C1_dDown = pad1.dpad_down;
                    C1_dLeft = pad1.dpad_left;
                    C2_dUp = pad2.dpad_up;
                    C2_dRight = pad2.dpad_right;
                    C2_dDown = pad2.dpad_down;
                    C2_dLeft = pad2.dpad_left;

                    sleep(50);
                }
            }
            catch (InterruptedException e) {}
            catch (Throwable e) {}
        }
    }
}