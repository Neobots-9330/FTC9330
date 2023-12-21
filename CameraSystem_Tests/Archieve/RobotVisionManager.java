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
    List<Recognition> RecognitionizedObjects; //Store instances of seen objects/regonitions from Tensor Flow; List of instances of class "Regonition", which stores information on the Regonition.
    OpMode opMode; //Instance of opMode, inialized when opMode is passed from teleop to RobotVisionManager.
    private TeamProp cubeProp; //Class stores data for tensorflow detection of our team prop.
    float teamPropconfidence = 0; //Confidence of tensor flow detection. 
    float teamPropLeft = 0; //Returns the left coordinate of the rectangle bounding the detected object.
    float teamPropRight = 0; //Returns the right coordinate of the rectangle bounding the detected object.
    float teamPropTop = 0; //Returns the top coordinate of the rectangle bounding the detected object.
    float teamPropBottom = 0; //Returns the bottom coordinate of the rectangle bounding the detected object.
    String teamPropLabel = ""; //Name of tensor flow object detected.
    
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
    
    //Returns the current regonitions detected.
    public void detectTeamProp() {
        
        RecognitionizedObjects = tfod_processor.getRecognitions(); //Array of regonized objects
        
        //Loop throught the regonized objects
        for (Recognition object : RecognitionizedObjects) {
            teamPropconfidence = object.getConfidence();
            teamPropLeft = object.getLeft();
            teamPropRight = object.getRight();
            teamPropBottom = object.getBottom();
            teamPropTop = object.getTop();
            teamPropLabel = object.getLabel();
        }
        
    }
}
