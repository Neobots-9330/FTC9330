package org.neobots2903.ftcCenterstage2023;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import java.time.Instant;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@TeleOp
public class TeleOp1 extends LinearOpMode {
    public boolean speedUpPressed = false;
    public boolean speedDownPressed = false;
    public boolean wheeling = false;
    int cycles; //Stores the number of "opModeIsActive" cycles; Total times the main infinite loop has ran, and completed.

    @Override
    public void runOpMode() throws InterruptedException {
        Robot9330 robot = new Robot9330(this, false);
        
        robot.initAprilTag(); //Iniatlize the aprilTag processor, get the camera up and running.

        //Print message to Driver Hub IOStream, showing that the bot is initalized.
        telemetry.addLine("Robot inialized, waiting for start...");
        telemetry.update();

        int maxSpeedMultiplier = 3;
        int speedMultiplier = (int) Math.ceil(maxSpeedMultiplier / 2.0);

        waitForStart();
        
        if(isStopRequested()) return;

        while(opModeIsActive()) {
            
            telemetry.addLine("Robot is active: True"); //Print message in Driver Hub IOStream, informing the user that the bot is running.
            
            //change speedMultiplier
            if(gamepad1.right_bumper && speedMultiplier < maxSpeedMultiplier && !speedUpPressed) {
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
            
            if(gamepad1.left_bumper && speedMultiplier > 1 && !speedDownPressed) {
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
            
            if(gamepad1.x) robot.imu.initialize(robot.parameters);
            
            double x = gamepad1.left_stick_x;
            double y = -gamepad1.left_stick_y;
            double rx = gamepad1.right_stick_x;

            robot.move(x * speedMultiplier / maxSpeedMultiplier, y * speedMultiplier / maxSpeedMultiplier, rx * speedMultiplier / maxSpeedMultiplier);
            
            /*if(gamepad1.a && !wheeling) {
                wheeling = true;
                
                new Thread() {
                    @Override
                    public void run() {
                        robot.servoWheelie.setPosition(1);
                        robot.pause(1);
                        robot.servoWheelie.setPosition(0);
                        wheeling = false;
                    }
                }.start();
            }*/
            
            //When "y" is pressed on gamepad2, launch the airplane.
            if (gamepad2.y) {
                robot.launchAirplane();
            }
            
            //If "b" is pressed on gamepad2, toggle the pixel trap up or down.
            if (gamepad2.b) {
                robot.togglePixelTrap();
                sleep(250);
            }

            //Print total number of main loop cycles.
            cycles++;
            telemetry.addLine("Total cycles: " + cycles);
            
            //Print message of wether a tag is seen or not to the Driver Hub IOStream.
            if (robot.aprilTagIsSeen() == true) {
                telemetry.addLine("April Tag Seen: True");
            } else {
                telemetry.addLine("April Tag Seen: False");
            }
            
            
            telemetry.update(); //Update Driver Hub IOStream.

        }
    }
}
