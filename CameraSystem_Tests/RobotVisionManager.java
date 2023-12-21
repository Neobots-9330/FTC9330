package org.neobots2903.ftcCenterstage2023;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import java.util.List;

/*
Class to manage Camera and vision.
*/
public class RobotVisionManager {
    
    private TfodProcessor tfod_processor; //Instance "TensorFlow Object Detection" software and processor.
    private VisionPortal cameraVisionPortal;
    List<Recognition> currentRecognitions; //Store instances of seen objects/regonitions from Tensor Flow; List of instances of class "Regonition", which stores information on the Regonition.
    OpMode opMode; //Instance of opMode, inialized when opMode is passed from teleop to RobotVisionManager.
    
    //Constructor.
    public RobotVisionManager(OpMode opMode) {
        this.opMode = opMode;
        tfod_processor = TfodProcessor.easyCreateWithDefaults(); //Init the tfod processor. Set it up the "Easy" way.
        cameraVisionPortal = VisionPortal.easyCreateWithDefaults(opMode.hardwareMap.get(WebcamName.class, "Webcam 1"), tfod_processor);
    }
    
    //Test
    /*public void printHi() {
        opMode.telemetry.addLine("Hello!");
        opMode.telemetry.update();
    }*/
    
    //Returns integer of how many object have been regonized by Tensor Flow.
    public int getTotalRegonizedObjects() {
        currentRecognitions = tfod_processor.getRecognitions();
        
        return currentRecognitions.size();
    }
}
