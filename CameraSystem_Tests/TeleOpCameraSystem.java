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
        int applicationCycles = 0; //Total cycles of the application.
        
        waitForStart();
        
        while (opModeIsActive()) {
            
            vision.detectTeamProp();
            telemetry.addLine("Left: " + Float.toString(vision.teamPropLeft));
            
            applicationCycles++;
            telemetry.addLine("Total cycles: " + applicationCycles);
            telemetry.update(); //Update telemetry after each application cycle.
        }
    }
}
