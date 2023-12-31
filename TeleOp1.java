package org.neobots2903.ftcCenterstage2023;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class TeleOp1 extends LinearOpMode {
    public boolean speedUpPressed = false;
    public boolean speedDownPressed = false;
    public boolean wheeling = false;
    int applicationCycles = 0; // Counts the total number of cycles of the application
    Debounce controllerButton_b;
    int sleepDebounceTime = 150;
    
    @Override
    public void runOpMode() throws InterruptedException {
        Robot9330 robot = new Robot9330(this, false);
        
        //Class to manage debounce
        controllerButton_b = new Debounce();

        int maxSpeedMultiplier = 3;
        int speedMultiplier = (int) Math.ceil(maxSpeedMultiplier / 2.0);

        waitForStart();

        if (isStopRequested())
            return;

        robot.initHanger(); // Set hanger encoder to 0.
        robot.initHangerExtender(); // Sets hanger extension positon to 0.

        while (opModeIsActive()) {
            // change speedMultiplier
            if (gamepad1.right_bumper && speedMultiplier < maxSpeedMultiplier && !speedUpPressed) {
                speedMultiplier++;
                speedUpPressed = true;

                new Thread() {
                    @Override
                    public void run() {
                        robot.pause(0.25);
                        speedUpPressed = false;
                    }
                }.start();
            }

            if (gamepad1.left_bumper && speedMultiplier > 1 && !speedDownPressed) {
                speedMultiplier--;
                speedDownPressed = true;

                new Thread() {
                    @Override
                    public void run() {
                        robot.pause(0.25);
                        speedDownPressed = false;
                    }
                }.start();
            }

            if (gamepad1.x)
                robot.imu.initialize(robot.parameters);

            double x = gamepad1.left_stick_x;
            double y = -gamepad1.left_stick_y;
            double rx = gamepad1.right_stick_x;

            robot.move(x * speedMultiplier / maxSpeedMultiplier, y * speedMultiplier / maxSpeedMultiplier,
                    rx * speedMultiplier / maxSpeedMultiplier);

            /*
             * if(gamepad1.a && !wheeling) {
             * wheeling = true;
             * 
             * new Thread() {
             * 
             * @Override
             * public void run() {
             * robot.servoWheelie.setPosition(1);
             * robot.pause(1);
             * robot.servoWheelie.setPosition(0);
             * wheeling = false;
             * }
             * }.start();
             * }
             */

            // When "y" is pressed on gamepad2, launch the airplane.
            if (gamepad2.y) {
                robot.launchAirplane();
            }
            
            // If "b" is pressed on gamepad2, toggle the pixel trap up or down.
            if (gamepad2.b) {
                robot.togglePixelTrap();
                sleep(sleepDebounceTime);
            }

            // If dpad up pressed, raise hanger.
            if (gamepad2.dpad_up) {
                robot.raiseHanger();
                sleep(sleepDebounceTime);
            }

            // If dpad down pressed, lower hanger.
            if (gamepad2.dpad_down) {
                robot.lowerHanger();
                sleep(sleepDebounceTime);
            }

            // If dpad left pressed, extend hanger extender.
            if (gamepad2.dpad_left) {
                robot.extendHangerExtender();
                sleep(sleepDebounceTime);
            }

            // If dpad right pressed, retract hanger extender.
            if (gamepad2.dpad_right) {
                robot.retractHangerExtender();
                sleep(sleepDebounceTime);
            }

            applicationCycles++; // Update total application cycles.
            telemetry.addData("Total cycles: ", applicationCycles);
            telemetry.addData("Hanger shoulder position: ", robot.getHangerPosition());
            telemetry.addData("Hanger extension position: ", robot.getHangerExtensionPosition());
            telemetry.addData("Pixel guard is down: ", robot.pixelTrapIsDown);

            telemetry.update(); // Update telemetry.
        }
    }
}
