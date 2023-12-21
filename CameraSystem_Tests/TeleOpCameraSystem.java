package org.neobots2903.ftcCenterstage2023;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@TeleOp

/*
TeleOp/Main class; Entry point.
*/
public class TeleOpCameraSystem extends LinearOpMode {
    RobotVisionManager vision; //Manages robots vision/camera.
    
    public static final boolean USE_WEBCAM = true;
    
    //Main/Entry point.
    public void runOpMode() {
        vision = new RobotVisionManager(this); //Manages robots vision/camera.
        
        
        waitForStart();
        
        while (opModeIsActive()) {
            
            telemetry.addLine("Tfod objects regonized: " + vision.getTotalRegonizedObjects());
            telemetry.update();
            
        }
    }
}
