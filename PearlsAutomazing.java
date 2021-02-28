package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.bosch.BNO055IMU;

/**
 * This code was based on https://github.com/nahn20/FGC_Guides/blob/master/driveJava.java
 * as described in paper: https://seas.yale.edu/sites/default/files/imce/other/HolonomicOmniWheelDrive.pdf
 * It's been modifed and documented to fit the needs to help teach our students and
 * used in our robot
 */
 
@Autonomous(name="PearlsAuto-mazing", group="2021UltimateGoal")
public class PearlsAutomazing extends LinearOpMode {

    private DcMotor frontLeftWheel  = null;
    private DcMotor backLeftWheel   = null;
    private DcMotor backRightWheel  = null;
    private DcMotor frontRightWheel = null;

    private DcMotor.Direction forward = DcMotor.Direction.FORWARD;
    private DcMotor.Direction reverse = DcMotor.Direction.REVERSE;

    private DcMotor.RunMode encoderSetting;
    private DcMotor.RunMode stopAndReset = DcMotor.RunMode.STOP_AND_RESET_ENCODER;
    private DcMotor.RunMode runToPosition = DcMotor.RunMode.RUN_TO_POSITION;
    private DcMotor.RunMode runUsingEncoder = DcMotor.RunMode.RUN_USING_ENCODER;

    // Tetrix Torquenado 44260, counts per revolution = 1440
    static final int CPR = 480;

    static final int DEFAULTINCHES = 1;
    static final int DEFAULTDEGREE = 1;
    static final int WHEELDIAMETER = 4;
    static final int ROBOTRADIUS = 9;

    double movementInInches = CPR / (WHEELDIAMETER * Math.PI);
    double movementPerDegree = ((ROBOTRADIUS / (WHEELDIAMETER / 2)) * CPR) / 360;
    double distance = 0.0;

    static final double STOPPOWER = 0.0;

    double shooterSpeed = 0;
    double maxShooterSpeed = 0.70;

    BNO055IMU imu;
    @Override
    public void runOpMode() {
        DcMotor collector;
        DcMotor shooter;

        leftDirection = forward;
        rightDirection = reverse;

        telemetry.setAutoClear(true);
        
        frontLeftWheel  = hardwareMap.dcMotor.get("frontLeftWheel");
        backLeftWheel   = hardwareMap.dcMotor.get("backLeftWheel");
        backRightWheel  = hardwareMap.dcMotor.get("backRightWheel");
        frontRightWheel = hardwareMap.dcMotor.get("frontRightWheel");

        collector = hardwareMap.dcMotor.get("collector");
        shooter = hardwareMap.dcMotor.get("shooter");

        frontLeftWheel.setDirection(leftDirection);
        backLeftWheel.setDirection(leftDirection);
        frontRightWheel.setDirection(rightDirection);
        backRightWheel.setDirection(rightDirection);

        frontLeftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        encoderSetting = stopAndReset;
        frontLeftWheel.setMode(encoderSetting);
        backLeftWheel.setMode(encoderSetting);
        frontRightWheel.setMode(encoderSetting);
        backRightWheel.setMode(encoderSetting);

        /**
         * RUN_USING_ENCODER is suppose to allow for more consistent power
         * to the motor.
         */
        shooter.setDirection(reverse);
        shooter.setMode(runUsingEncoder);
        shooter.setMaxSpeed(maxShooterSpeed);

        // Wait for the start of the game
        waitForStart();
       
        moveForwardOrBack((-7 * 12), 0.15);
        
        moveForwardOrBack(42, 0.25);
        
        strafeLeftOrRight(-16, 0.25);
        
        rotateLeftOrRight(-11, 0.25);

        //shooter
        shooterSpeed = maxShooterSpeed;
        shooter.setPower(shooterSpeed);
        
        sleep(1000);
            
        collector.setPower(1.0);
        
        sleep(3000);
        
        collector.setPower(0.0);
        shooter.setPower(0.0);
       
        moveForwardOrBack(-10, 0.25);
    }

    /** 
     * moveForwardOrBack()
     * 
     * Move robot forward or back
     * 
     * @params int distanceInInches Postive value moves forward, negative value back
     * @params double speed Range 0.0 to 1.0
     */
    private void moveForwardOrBack(int distanceInInches, double speed) {
        // Get current motor positions
        int frontLeftPosition = frontLeftWheel.getCurrentPosition();
        int backLeftPosition = backLeftWheel.getCurrentPosition();
        int frontRightPosition = frontRightWheel.getCurrentPosition();
        int backRightPosition = backRightWheel.getCurrentPosition();
        double movement = distanceInInches * movementInInches;

        // Apply movement
        frontLeftPosition += movement;
        backLeftPosition += movement;
        frontRightPosition += movement;
        backRightPosition += movement;

        // Set location to move to
        frontLeftWheel.setTargetPosition(frontLeftPosition);
        backLeftWheel.setTargetPosition(backLeftPosition);
        frontRightWheel.setTargetPosition(frontRightPosition);
        backRightWheel.setTargetPosition(backRightPosition);

        encoderSetting = runToPosition;
        frontLeftWheel.setMode(encoderSetting);
        backLeftWheel.setMode(encoderSetting);
        frontRightWheel.setMode(encoderSetting);
        backRightWheel.setMode(encoderSetting);
        
        // Start moving robot by setting motor speed
        frontLeftWheel.setPower(speed);
        backLeftWheel.setPower(speed);
        frontRightWheel.setPower(speed);
        backRightWheel.setPower(speed);

        while (frontLeftWheel.isBusy()
            && backLeftWheel.isBusy()
            && frontRightWheel.isBusy()
            && backRightWheel.isBusy()
        ) {
            sendTelemetry();
            idle();
        }

        frontLeftWheel.setPower(STOPPOWER);
        backLeftWheel.setPower(STOPPOWER);
        frontRightWheel.setPower(STOPPOWER);
        backRightWheel.setPower(STOPPOWER);
        
        encoderSetting = stopAndReset;
        frontLeftWheel.setMode(encoderSetting);
        backLeftWheel.setMode(encoderSetting);
        frontRightWheel.setMode(encoderSetting);
        backRightWheel.setMode(encoderSetting);
    }

    /** 
     * strafeRightLeft()
     * 
     * Strafe robot left or right 
     * 
     * @params int distanceInInches Postive value moves right, negative value left
     * @params double speed Range 0.0 to 1.0
     */
    private void strafeLeftOrRight(int distanceInInches, double speed) {

        // Get current motor positions
        int frontLeftPosition = frontLeftWheel.getCurrentPosition();
        int backLeftPosition = backLeftWheel.getCurrentPosition();
        int frontRightPosition = frontRightWheel.getCurrentPosition();
        int backRightPosition = backRightWheel.getCurrentPosition();
        double movement = distanceInInches * movementInInches;

        // Apply movement
        frontLeftPosition += movement;
        backLeftPosition -= movement;
        frontRightPosition -= movement;
        backRightPosition += movement;

        // Set location to move to
        frontLeftWheel.setTargetPosition(frontLeftPosition);
        backLeftWheel.setTargetPosition(backLeftPosition);
        frontRightWheel.setTargetPosition(frontRightPosition);
        backRightWheel.setTargetPosition(backRightPosition);

        encoderSetting = runToPosition;
        frontLeftWheel.setMode(encoderSetting);
        backLeftWheel.setMode(encoderSetting);
        frontRightWheel.setMode(encoderSetting);
        backRightWheel.setMode(encoderSetting);
        
        // Start moving robot by setting motor speed
        frontLeftWheel.setPower(speed);
        backLeftWheel.setPower(speed);
        frontRightWheel.setPower(speed);
        backRightWheel.setPower(speed);

        while (frontLeftWheel.isBusy()
            && backLeftWheel.isBusy()
            && frontRightWheel.isBusy()
            && backRightWheel.isBusy()
        ) {
            sendTelemetry();
            idle();
        }

        frontLeftWheel.setPower(STOPPOWER);
        backLeftWheel.setPower(STOPPOWER);
        frontRightWheel.setPower(STOPPOWER);
        backRightWheel.setPower(STOPPOWER);
        
        encoderSetting = stopAndReset;
        frontLeftWheel.setMode(encoderSetting);
        backLeftWheel.setMode(encoderSetting);
        frontRightWheel.setMode(encoderSetting);
        backRightWheel.setMode(encoderSetting);
    }

    /** 
     * rotateLeftOrRight()
     * 
     * Rotate angle left or right
     * 
     * @params int rotateAngle Postive value rotates right, negative rotates left
     * @params double speed Range 0.0 to 1.0
     */
    private void rotateLeftOrRight(int rotateAngle, double speed) {

        // Get current motor positions
        int frontLeftPosition = frontLeftWheel.getCurrentPosition();
        int backLeftPosition = backLeftWheel.getCurrentPosition();
        int frontRightPosition = frontRightWheel.getCurrentPosition();
        int backRightPosition = backRightWheel.getCurrentPosition();
        double movement = rotateAngle * movementPerDegree;

        // Apply movement
        frontLeftPosition += movement;
        backLeftPosition += movement;
        frontRightPosition -= movement;
        backRightPosition -= movement;

        // Set location to move to
        frontLeftWheel.setTargetPosition(frontLeftPosition);
        backLeftWheel.setTargetPosition(backLeftPosition);
        frontRightWheel.setTargetPosition(frontRightPosition);
        backRightWheel.setTargetPosition(backRightPosition);

        encoderSetting = runToPosition;
        frontLeftWheel.setMode(encoderSetting);
        backLeftWheel.setMode(encoderSetting);
        frontRightWheel.setMode(encoderSetting);
        backRightWheel.setMode(encoderSetting);
        
        // Start moving robot by setting motor speed
        frontLeftWheel.setPower(speed);
        backLeftWheel.setPower(speed);
        frontRightWheel.setPower(speed);
        backRightWheel.setPower(speed);

        while (frontLeftWheel.isBusy()
            && backLeftWheel.isBusy()
            && frontRightWheel.isBusy()
            && backRightWheel.isBusy()
        ) {
            sendTelemetry();
            idle();
        }

        frontLeftWheel.setPower(STOPPOWER);
        backLeftWheel.setPower(STOPPOWER);
        frontRightWheel.setPower(STOPPOWER);
        backRightWheel.setPower(STOPPOWER);
        
        encoderSetting = stopAndReset;
        frontLeftWheel.setMode(encoderSetting);
        backLeftWheel.setMode(encoderSetting);
        frontRightWheel.setMode(encoderSetting);
        backRightWheel.setMode(encoderSetting);

    }

    private void sendTelemetry() {
        telemetry.addData(
            "Target", 
            "%7d :%7d :%7d :%7d", 
            frontLeftPosition, 
            frontRightPosition, 
            backLeftPosition, 
            backRightPosition);
        telemetry.addData(
            "Actual", 
            "%7d :%7d :%7d :%7d", 
            frontLeftWheel.getCurrentPosition(),
            frontRightWheel.getCurrentPosition(), 
            backLeftWheel.getCurrentPosition(),
            backRightWheel.getCurrentPosition()
            );

        telemetry.update();
    }
}