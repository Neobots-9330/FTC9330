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
    
    
    public Robot9330(OpMode opMode, boolean flip) {
        this.opMode = opMode;
        this.flip = flip;
        imu = opMode.hardwareMap.get(BNO055IMU.class, "imu");
        motorDriveFrontLeft = opMode.hardwareMap.get(DcMotor.class, "motorDriveFrontLeft");
        motorDriveFrontRight = opMode.hardwareMap.get(DcMotor.class, "motorDriveFrontRight");
        motorDriveBackLeft = opMode.hardwareMap.get(DcMotor.class, "motorDriveBackLeft");
        motorDriveBackRight = opMode.hardwareMap.get(DcMotor.class, "motorDriveBackRight");
        //servoWheelie = opMode.hardwareMap.get(Servo.class, "servoWheelie");
        /*airplaneLauncherRelease = opMode.hardwareMap.get(Servo.class, "servoDrone");
        pixelTrapServoOne = opMode.hardwareMap.get(Servo.class, "pixelTrapServoOne");
        pixelTrapServoTwo = opMode.hardwareMap.get(Servo.class, "pixelTrapServoTwo");*/
        
        
        //airplaneLauncherRelease.setPosition(0.64); //Lock in the rubber band; Removed, moves the servo to much.
        
        //Set up IMU
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);
        
        /*
        
        WARNING: LAST YEARS HALF-ASSED CODE, IF MOTORS NEED TO BE REVERSED, REVERSE IT HERE!
        
        */
        // Reverse any motors if necessary
        //motorDriveFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorDriveFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorDriveBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        //motorDriveBackRight.setDirection(DcMotorSimple.Direction.REVERSE);
        // motorLiftRight.setDirection(DcMotorSimple.Direction.REVERSE);
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
        double frontRightPower = (rotY - rotX - rx) / denominator; //MOTOR is flipped.
        double backLeftPower = (rotY - rotX + rx) / denominator;
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
    
    
    //Tick is what the motor encoders return.
    //One wheel rotation = 280 ticks = 23.93 centimeters.
    //Converts inch to ticks for driving forward and back.
    public int toTicks_ForwardAndBack(double inch) {
        return (int) Math.ceil(inch * 29.71);
    }
    
    //Converts inch to ticks for driving left and right (Strafe).
    public int toTicks_LeftAndRight(double inch) {
        
    }
    
    //Fully resets encoders.
    public void runBackEncoders() {
        motorDriveFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorDriveFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorDriveBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorDriveBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    
    //Moves robot forward for a specified number of ticks; power is the motor power.
    public void moveForwardOrReverse(int ticks, double power) {
        runBackEncoders(); //Reset encoders.
        
        motorDriveFrontLeft.setTargetPosition(ticks); //Set target position for all motors.
        motorDriveFrontRight.setTargetPosition(ticks);
        motorDriveBackLeft.setTargetPosition(ticks);
        motorDriveBackRight.setTargetPosition(ticks);
        
        motorDriveFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION); //Drive the motor to the encoder position.
        motorDriveFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorDriveBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorDriveBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        //1st argument is ticks per second. (distance traveled)
        motorDriveFrontLeft.setPower(power);
        motorDriveFrontRight.setPower(power);
        motorDriveBackLeft.setPower(power);
        motorDriveBackRight.setPower(power * -1);
    }
    
    //One inch = 32 ticks.
    //Strafe Right: Front left > Forwards; Rear Left > Backwards; Front right > backwards; rear right > forwards.
    //Strafe Left: Front left > Backwards; Rear left > Forwards; Front Right > Forwards; Rear right > backwards.
    //Strafe the robot sideways for a number of ticks.
    //"direction_id"; 0 == left, 1 == right;
    public void strafe(int ticks, double power, int direction_id) {
        runBackEncoders();
        
        if (direction_id == 0) {
            
        }
        
        
        
        if (direction_id == 1) {
            
        }
        
    }
    
    public void pause(double seconds) {
        try {
            Thread.sleep((long) (seconds * 1000));
        } catch(InterruptedException exc) {}
    }
    
    
    public void autoBB() {
        
        moveForwardOrReverse(toTicks_ForwardAndBack(12.0), 0.2);
        
        //auto!!! 
        //forward - back - left
    }
    
    
    public void autoBW() {
        
        move(0, .5, 0, .67);
        move(0, 0, 0, .5);
        move(0, -.5, 0, .59);
        move(0, 0, 0, .5);
        move(-0.5, 0, 0, 2.75);
        
        //auto!!! 
        //forward - back - left
    }
    
    
    
    public void autoRB() {
        
        move(0, .5, 0, .67);
        move(0, 0, 0, .5);
        move(0, -.5, 0, .5);
        move(0, 0, 0, .5);
        move(0.5, 0, 0, 1.2);
        
        //auto!!! 
        //forward - back - left
    }
    
    
    public void autoRW() {
        
        move(0, .5, 0, .67);
        move(0, 0, 0, .5);
        move(0, -.5, 0, .56); //Orignally was .59 sec for time.
        move(0, 0, 0, .5);
        move(0.5, 0, 0, 2.75);
        
        //auto!!! 
        //forward - back - left
    }
    
    //returns current front left motor position in ticks.
    public int getFrontLeftMotorPosition() {
        return motorDriveFrontLeft.getCurrentPosition();
    }
}


/*
Plane launcher notes:
line 20, 21, 32, 83-89 commented out for now.
*/

/*280 ticks = one full rotation.*/

//One wheel rotation = 280 ticks = 23.93 centimeters.
    //Method to test ticks.
    /*public void tickTest() {
        
        motorDriveFrontLeft.setTargetPosition(500);
        
        if (motorDriveFrontLeft.getTargetPosition() > motorDriveFrontLeft.getCurrentPosition()) {
            motorDriveFrontLeft.setPower(0.2);
        }
    }*/
    
    /*motorDriveFrontLeft_ticks = motorDriveFrontLeft.getTargetPosition();
        motorDriveFrontRight_ticks = motorDriveFrontRight.getTargetPosition();
        motorDriveBackLeft_ticks = motorDriveBackLeft.getTargetPosition();
        motorDriveBackRight_ticks = motorDriveBackRight.getTargetPosition();*/
        
        /*move(0, .5, 0, .67);
        move(0, 0, 0, .5);
        move(0, -.5, 0, .5);
        move(0, 0, 0, .5);
        move(-0.5, 0, 0, 1.2);*/
