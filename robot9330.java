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
        
        // Reverse any motors if necessary
        //motorDriveFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorDriveFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorDriveBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorDriveBackRight.setDirection(DcMotorSimple.Direction.REVERSE);
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
        double backRightPower = -(rotY + rotX - rx) / denominator;
        
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
            airplaneLauncherRelease.setPosition(0); //NOTES: 0.64 to hold down band.
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
    
    public void autoBB() {
        
        move(0, .5, 0, .67);
        move(0, 0, 0, .5);
        move(0, -.5, 0, .5);
        move(0, 0, 0, .5);
        move(-0.5, 0, 0, 1.2);
        
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
