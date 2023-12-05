package org.neobots2903.ftcCenterstage2023;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Robot9330 {
    public BNO055IMU imu;
    public BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
    public DcMotor motorDriveFrontLeft; //Front Left motor for wheel base.
    public DcMotor motorDriveFrontRight; //Front Right motor for wheel base.
    public DcMotor motorDriveBackLeft; //Rear Left motor for wheel base.
    public DcMotor motorDriveBackRight; //Rear Right motor for wheel base.
    public OpMode opMode;
    //public Servo servoWheelie;
    public boolean flip;
    public Servo airplaneLauncherRelease; //The servo that release the airplane on the launcher.
    boolean planeIsLaunched = false; //Switched to true when the plane is launched.
    public Servo pixelTrapServoOne; //First Servo for the pixel trap.
    public Servo pixelTrapServoTwo; //Second Servo for the pixel trap.
    boolean pixelTrapIsDown = false;
    public DcMotor motorHangShoulder;
    public DcMotor motorHangArm;
    
    //Reverse motors for atonnymus
    boolean atonnymus_motorDriveFrontLeft_reverse = false;
    boolean atonnymus_motorDriveFrontRight_reverse = true;
    boolean atonnymus_motorDriveBackLeft_reverse = true;
    boolean atonnymus_motorDriveBackRight_reverse = false;
    
    //"Magic numbers"
    int hanger_upright_position = 175; //Total ticks to raise the hanger arm to upright position.
    int hanger_extender_position = 3450; //Total ticks to move the hanger actuator upward to grip rigging.
    double ticksToMoveForwardOneInch = 29.71;
    int ticksToStrafOneInch = 30;
    
    //Auto variables are all in Inches.
    double autoForward = 30.75; //Distance to move forward toward strip for all autos. //Others: 32.75
    double autoBackward = 26.75; //Distance to move backward away from strip for all autos, alligning between rigging to starfe for backdrop park.//Previous 28.75
    double autoWing = 98 + 2; //Distance to strafe from the wing to backdrop park.
    double autoBackdrop = 50.5; //Distance to strafe from the Backdrop start to backdrop park.
    double autoPower = 0.2;
    
    public Robot9330(OpMode opMode, boolean flip) {
        this.opMode = opMode;
        this.flip = flip;
        imu = opMode.hardwareMap.get(BNO055IMU.class, "imu");
        motorDriveFrontLeft = opMode.hardwareMap.get(DcMotor.class, "motorDriveFrontLeft");
        motorDriveFrontRight = opMode.hardwareMap.get(DcMotor.class, "motorDriveFrontRight");
        motorDriveBackLeft = opMode.hardwareMap.get(DcMotor.class, "motorDriveBackLeft");
        motorDriveBackRight = opMode.hardwareMap.get(DcMotor.class, "motorDriveBackRight");
        //servoWheelie = opMode.hardwareMap.get(Servo.class, "servoWheelie");
        airplaneLauncherRelease = opMode.hardwareMap.get(Servo.class, "servoDrone");
        pixelTrapServoOne = opMode.hardwareMap.get(Servo.class, "pixelTrapServoOne"); //left servo.
        pixelTrapServoTwo = opMode.hardwareMap.get(Servo.class, "pixelTrapServoTwo"); //right servo
        motorHangShoulder = opMode.hardwareMap.get(DcMotor.class, "motorHangShoulder");
        motorHangArm = opMode.hardwareMap.get(DcMotor.class, "motorHangArm");
        //airplaneLauncherRelease.setPosition(0.64); //Lock in the rubber band; Removed, moves the servo to much.
        
        //Set up IMU
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);
        
        //Set brake on hang arm.
        motorHangArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorHangShoulder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Reverse any motors if necessary
        //motorDriveFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorDriveFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorDriveBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorDriveBackRight.setDirection(DcMotorSimple.Direction.REVERSE);
        // motorLiftRight.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    
    //Inailizes the hangers 0 tick position to its current starting position.
    public void initHanger() {
        motorHangShoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    
    //Iniatlises the hangers extension. Sets its encoder to 0.
    public void initHangerExtender() {
        motorHangArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    
    public int getHangerExtensionPosition() {
        return motorHangArm.getCurrentPosition();
    }
    
    //Raises the hanger to its upward position. 164 is upright.
    public void raiseHanger() {
        motorHangShoulder.setTargetPosition(hanger_upright_position);
        motorHangShoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorHangShoulder.setPower(0.05);
        
    }
    
    public void lowerHanger() {
        motorHangShoulder.setTargetPosition(0);
        motorHangShoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorHangShoulder.setPower(-0.05);
    }
    
    //Extends hanger extender upward.//4189 is default
    public void extendHangerExtender() {
        motorHangArm.setTargetPosition(hanger_extender_position);
        motorHangArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorHangArm.setPower(0.4);
    }
    
    public void retractHangerExtender() {
        motorHangArm.setTargetPosition(0);
        motorHangArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorHangArm.setPower(-1);
    }
    
    
    //Returns the hangers position in ticks.
    public int getHangerPosition() {
        return motorHangShoulder.getCurrentPosition();
    }
    
    
    // Moves robot
    public void move(double x, double y, double rx) {
        if(flip) {
            x = -x;
            rx = -rx;
        }
        
        // Magical math
        double botHeading = -imu.getAngularOrientation().firstAngle;
        double rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
        double rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double frontRightPower = -(rotY - rotX - rx) / denominator; //MOTOR is flipped.
        double backLeftPower = -(rotY - rotX + rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;
        
        // Move motors
        motorDriveFrontLeft.setPower(frontLeftPower);
        motorDriveFrontRight.setPower(frontRightPower);
        motorDriveBackLeft.setPower(backLeftPower);
        motorDriveBackRight.setPower(backRightPower);
    }
    
    // Moves robot for an amount of time
    public void move(double x, double y, double rx, double seconds) {
        move(x, y, rx);
        pause(seconds);
        move(0, 0, 0);
    }
    
    public void pause(double seconds) {
        try {
            Thread.sleep((long) (seconds * 1000));
        } catch(InterruptedException exc) {}
    }
    
    //Rotates a servo, releasing the paper airplane.
    public void launchAirplane() {
        if (planeIsLaunched == false) { //Servo can only be rotated once.
            airplaneLauncherRelease.setPosition(1);
            planeIsLaunched = true;
        }
    }

    //Moves the pixel trap up if down, and downif up.
    public void togglePixelTrap() {
        if (pixelTrapIsDown == false) {
            pixelTrapServoOne.setPosition(1);
            pixelTrapServoTwo.setPosition(1);
            pixelTrapIsDown = true;
        } else if (pixelTrapIsDown == true) {
            pixelTrapServoOne.setPosition(0);
            pixelTrapServoTwo.setPosition(0);
            pixelTrapIsDown = false;
        }
    }
    
    //Tick is what the motor encoders return.
    //One wheel rotation = 280 ticks = 23.93 centimeters.
    //Converts inch to ticks for driving forward and back.
    public int toTicks_ForwardAndBack(double inch) {
        return (int) Math.ceil(inch * ticksToMoveForwardOneInch);
    }
    
    //Converts inch to ticks for driving left and right (Strafe).
    public int toTicks_LeftAndRight(double inch) {
        return (int) Math.ceil(inch * ticksToStrafOneInch);
    }
    
    //Fully resets encoders.
    public void runBackEncoders() {
        motorDriveFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorDriveFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorDriveBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorDriveBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    
    
    //Moves robot forward for a specified number of ticks; power is the motor power.
    //Direction id: 0 = forward; 1 = backward;
    public void moveForwardOrReverse(int ticks, double power, int direction_id) {
        runBackEncoders(); //Reset encoders.
        
        //Move forward.
        if (direction_id == 0) {
            //Set motor target positions to forward.
            
            if (atonnymus_motorDriveBackRight_reverse == false) {
                motorDriveBackRight.setTargetPosition(ticks);
                motorDriveBackRight.setPower(power); //Werid motor.
            } else {
                motorDriveBackRight.setTargetPosition(-ticks);
                motorDriveBackRight.setPower(-power); //Werid motor.
            }
            
            if (atonnymus_motorDriveBackLeft_reverse == false) {
                motorDriveBackLeft.setTargetPosition(ticks);
                motorDriveBackLeft.setPower(power);
            } else {
                motorDriveBackLeft.setTargetPosition(-ticks);
                motorDriveBackLeft.setPower(-power);
            }
            
            if (atonnymus_motorDriveFrontRight_reverse == false) {
                motorDriveFrontRight.setTargetPosition(ticks);
                motorDriveFrontRight.setPower(power);
            } else {
                motorDriveFrontRight.setTargetPosition(-ticks);
                motorDriveFrontRight.setPower(-power);
            }
            
            if (atonnymus_motorDriveFrontLeft_reverse == false) {
                motorDriveFrontLeft.setTargetPosition(ticks);
                motorDriveFrontLeft.setPower(power);
            } else {
                motorDriveFrontLeft.setTargetPosition(-ticks);
                motorDriveFrontLeft.setPower(-power);
            }
            
        }
        
        //Move backward
        if (direction_id == 1) {
            
            if (atonnymus_motorDriveBackRight_reverse == false) {
                motorDriveBackRight.setTargetPosition(-ticks);
                motorDriveBackRight.setPower(-power); //Werid motor.
            } else {
                motorDriveBackRight.setTargetPosition(ticks);
                motorDriveBackRight.setPower(power); //Werid motor.
            }
            
            if (atonnymus_motorDriveBackLeft_reverse == false) {
                motorDriveBackLeft.setTargetPosition(-ticks);
                motorDriveBackLeft.setPower(-power);
            } else {
                motorDriveBackLeft.setTargetPosition(ticks);
                motorDriveBackLeft.setPower(power);
            }
            
            if (atonnymus_motorDriveFrontRight_reverse == false) {
                motorDriveFrontRight.setTargetPosition(-ticks);
                motorDriveFrontRight.setPower(-power);
            } else {
                motorDriveFrontRight.setTargetPosition(ticks);
                motorDriveFrontRight.setPower(power);
            }
            
            if (atonnymus_motorDriveFrontLeft_reverse == false) {
                motorDriveFrontLeft.setTargetPosition(-ticks);
                motorDriveFrontLeft.setPower(-power);
            } else {
                motorDriveFrontLeft.setTargetPosition(ticks);
                motorDriveFrontLeft.setPower(power);
            }
        }
        
        motorDriveFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION); //Drive the motor to the encoder position.
        motorDriveFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorDriveBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorDriveBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
    }
    
    
    //One inch = 32 ticks.
    //Strafe Right: Front left > Forwards; Rear Left > Backwards; Front right > backwards; rear right > forwards.
    //Strafe Left: Front left > Backwards; Rear left > Forwards; Front Right > Forwards; Rear right > backwards.
    //Strafe the robot sideways for a number of ticks.
    //"direction_id"; 0 == left, 1 == right;
    public void strafe(int ticks, double power, int direction_id) {
        runBackEncoders(); //Set encoders to 0.
        
        //Strafe left
        if (direction_id == 0) {
            //Set motor target positions for Strafing left.
            
            if (atonnymus_motorDriveFrontLeft_reverse == false) {
                motorDriveFrontLeft.setTargetPosition(-(ticks));
                motorDriveFrontLeft.setPower(-power);
            } else {
                motorDriveFrontLeft.setTargetPosition(ticks);
                motorDriveFrontLeft.setPower(power);
            }
            
            if (atonnymus_motorDriveFrontRight_reverse == false) {
                motorDriveFrontRight.setTargetPosition(ticks);
                motorDriveFrontRight.setPower(power);
            } else {
                motorDriveFrontRight.setTargetPosition(-ticks);
                motorDriveFrontRight.setPower(-power);
            }
            
            if (atonnymus_motorDriveBackLeft_reverse == false) {
                motorDriveBackLeft.setTargetPosition(ticks);
                motorDriveBackLeft.setPower(power);
            } else {
                motorDriveBackLeft.setTargetPosition(-ticks);
                motorDriveBackLeft.setPower(-power);
            }
            
            if (atonnymus_motorDriveBackRight_reverse == false) {
                motorDriveBackRight.setTargetPosition(-(ticks));
                motorDriveBackRight.setPower(-power);
            } else {
                motorDriveBackRight.setTargetPosition(ticks);
                motorDriveBackRight.setPower(power);
            }
            
            motorDriveFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorDriveFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorDriveBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorDriveBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            
        }
    
        //Strafe Right
        if (direction_id == 1) {
            
            if (atonnymus_motorDriveFrontLeft_reverse == false) {
                motorDriveFrontLeft.setTargetPosition(ticks);
                motorDriveFrontLeft.setPower(power);
            } else {
                motorDriveFrontLeft.setTargetPosition(-ticks);
                motorDriveFrontLeft.setPower(-power);
            }
            
            if (atonnymus_motorDriveFrontRight_reverse == false) {
                motorDriveFrontRight.setTargetPosition(-ticks);
                motorDriveFrontRight.setPower(-power);
            } else {
                motorDriveFrontRight.setTargetPosition(ticks);
                motorDriveFrontRight.setPower(power);
            }
            
            if (atonnymus_motorDriveBackLeft_reverse == false) {
                motorDriveBackLeft.setTargetPosition(-ticks);
                motorDriveBackLeft.setPower(-power);
            } else {
                motorDriveBackLeft.setTargetPosition(ticks);
                motorDriveBackLeft.setPower(power);
            }
            
            if (atonnymus_motorDriveBackRight_reverse == false) {
                motorDriveBackRight.setTargetPosition(ticks);
                motorDriveBackRight.setPower(power);
            } else {
                motorDriveBackRight.setTargetPosition(-ticks);
                motorDriveBackRight.setPower(-power);
            }
            
            motorDriveFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorDriveFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorDriveBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorDriveBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        
    }
    
    //Wait for "motorDriveFrontRight" (Front right motor) to reach its encoder before entering the next atonnymus stage.
    public void waitForMotorEncoderToFinnish() {
        while(motorDriveFrontRight.isBusy()) {
            
        }
    }
    
    public void autoBB() {
        
                                                    //Distance \ power \ direction.
        moveForwardOrReverse(toTicks_ForwardAndBack(autoForward), autoPower, 0);
        waitForMotorEncoderToFinnish();
        moveForwardOrReverse(toTicks_ForwardAndBack(autoBackward), autoPower, 1);
        waitForMotorEncoderToFinnish();
        strafe(toTicks_LeftAndRight(autoBackdrop), autoPower, 0);
        waitForMotorEncoderToFinnish();
        
        
    }
    
    
    public void autoBW() {
        
        moveForwardOrReverse(toTicks_ForwardAndBack(autoForward), autoPower, 0);
        waitForMotorEncoderToFinnish();
        moveForwardOrReverse(toTicks_ForwardAndBack(autoBackward), autoPower, 1);
        waitForMotorEncoderToFinnish();
        strafe(toTicks_LeftAndRight(autoWing), autoPower, 0);
        waitForMotorEncoderToFinnish();
    }
    
    
    
    public void autoRB() {
        
        moveForwardOrReverse(toTicks_ForwardAndBack(autoForward), autoPower, 0);
        waitForMotorEncoderToFinnish();
        moveForwardOrReverse(toTicks_ForwardAndBack(autoBackward), autoPower, 1);
        waitForMotorEncoderToFinnish();
        strafe(toTicks_LeftAndRight(autoBackdrop), autoPower, 1);
        waitForMotorEncoderToFinnish();
    }
    
    
    public void autoRW() {
        
        moveForwardOrReverse(toTicks_ForwardAndBack(autoForward), autoPower, 0);
        waitForMotorEncoderToFinnish();
        moveForwardOrReverse(toTicks_ForwardAndBack(autoBackward), autoPower, 1);
        waitForMotorEncoderToFinnish();
        strafe(toTicks_LeftAndRight(autoWing), autoPower, 1);
        waitForMotorEncoderToFinnish();
        
    }
}


/*
Plane launcher notes:
line 20, 21, 32, 83-89 commented out for now.
*/


/*//Set target positions to negative (backwards).
            motorDriveFrontLeft.setTargetPosition(-(ticks)); //Set target position for all motors.
            motorDriveFrontRight.setTargetPosition(-(ticks));
            motorDriveBackLeft.setTargetPosition(-(ticks));
            motorDriveBackRight.setTargetPosition(-(ticks));
        
            //Set motor power to reverse.
            motorDriveFrontLeft.setPower(-power);
            motorDriveFrontRight.setPower(-power);
            motorDriveBackLeft.setPower(-power);
            motorDriveBackRight.setPower(-power); //Werid motor.*/

/*//Set motor target positions for Strafing left.
            motorDriveFrontLeft.setTargetPosition(-(ticks));
            motorDriveFrontRight.setTargetPosition(ticks);
            motorDriveBackLeft.setTargetPosition(ticks);
            motorDriveBackRight.setTargetPosition(-(ticks)); //Motor is possibly backwards
            
            motorDriveFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION); //Drive the motor to the encoder position.
            motorDriveFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorDriveBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorDriveBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            
            motorDriveFrontLeft.setPower(-power);
            motorDriveFrontRight.setPower(power);
            motorDriveBackLeft.setPower(power);
            motorDriveBackRight.setPower(-power); //Motor is somehow backwards.*/
            
            
            /*//Set motor target positions for Strafing left.
            motorDriveFrontLeft.setTargetPosition(ticks);
            motorDriveFrontRight.setTargetPosition(-(ticks));
            motorDriveBackLeft.setTargetPosition(-(ticks));
            motorDriveBackRight.setTargetPosition(ticks); //Motor is possibly backwards
            
            motorDriveFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION); //Drive the motor to the encoder position.
            motorDriveFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorDriveBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorDriveBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            
            motorDriveFrontLeft.setPower(power);
            motorDriveFrontRight.setPower(-power);
            motorDriveBackLeft.setPower(-power);
            motorDriveBackRight.setPower(power); //Motor is somehow backwards.*/
