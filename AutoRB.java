package org.neobots2903.ftcCenterstage2023;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous
public class AutoRB extends LinearOpMode {
    
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        
        if(opModeIsActive()) {
            new Robot9330(this, false).autoRB();
            
            while(opModeIsActive()) {};
        }
    }
}
