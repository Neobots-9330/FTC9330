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
    
    @Override
    public void runOpMode() throws InterruptedException {
        Robot9330 robot = new Robot9330(this, false);
        
        int maxSpeedMultiplier = 3;
        int speedMultiplier = (int) Math.ceil(maxSpeedMultiplier / 2.0);

        waitForStart();
        
        if(isStopRequested()) return;
        

        while(opModeIsActive()) {
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
            
            telemetry.update(); //Update telemetry each loop.
        }
    }
}
