package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

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

    // Start Change
    private DcMotor.Direction forward;
    private DcMotor.Direction reverse;
    private DcMotor.Direction leftDirection;
    private DcMotor.Direction rightDirection;

    private DcMotor.RunMode encoderSetting;
    private DcMotor.RunMode stopAndReset = DcMotor.RunMode.STOP_AND_RESET_ENCODER;
    private DcMotor.RunMode runToPosition = DcMotor.RunMode.RUN_TO_POSITION;
    private DcMotor.RunMode runUsingEncoder = DcMotor.RunMode.RUN_USING_ENCODER;

    // Tetrix Torquenado 44260, counts per revolution = 1440
    static final int cpr = 480;

    static final int defaultInches = 1;
    static final int defaultDegree = 1;
    static final int wheelDiameter = 4;
    static final int robotRadius = 9;
    double movementInInches = cpr / (wheelDiameter * Math.PI);
    // double movementInInches = cpr;
    double movementPerDegree = ((robotRadius / (wheelDiameter / 2)) * cpr) / 360;
    double distance = 0.0;

    static final double stopPower = 0.0;
    // End Change
    private DcMotor collector;
    private DcMotor shooter;
    double shooterSpeed;
    double maxShooterSpeed;

    BNO055IMU imu;
    @Override
    public void runOpMode() {
        telemetry.setAutoClear(true);
        
        frontLeftWheel  = hardwareMap.dcMotor.get("frontLeftWheel");
        backLeftWheel   = hardwareMap.dcMotor.get("backLeftWheel");
        backRightWheel  = hardwareMap.dcMotor.get("backRightWheel");
        frontRightWheel = hardwareMap.dcMotor.get("frontRightWheel");
        collector = hardwareMap.dcMotor.get("collector");
        shooter = hardwareMap.dcMotor.get("shooter");

        // Start Changes
        // Doing it this way will allow changing wheel direction faster
        forward = DcMotor.Direction.FORWARD;
        reverse = DcMotor.Direction.REVERSE;
        leftDirection = forward;
        rightDirection = reverse;

        frontLeftWheel.setDirection(leftDirection);
        backLeftWheel.setDirection(leftDirection);
        frontRightWheel.setDirection(rightDirection);
        backRightWheel.setDirection(rightDirection);

        // End Changes

        frontLeftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Start Changes
        // Encoder functionality
        stopAndReset = DcMotor.RunMode.STOP_AND_RESET_ENCODER;
        runToPosition = DcMotor.RunMode.RUN_TO_POSITION;
        runUsingEncoder = DcMotor.RunMode.RUN_USING_ENCODER;

        encoderSetting = stopAndReset;
        frontLeftWheel.setMode(encoderSetting);
        backLeftWheel.setMode(encoderSetting);
        frontRightWheel.setMode(encoderSetting);
        backRightWheel.setMode(encoderSetting);

        /*
        encoderSetting = runToPosition;
        frontLeftWheel.setMode(encoderSetting);
        backLeftWheel.setMode(encoderSetting);
        frontRightWheel.setMode(encoderSetting);
        backRightWheel.setMode(encoderSetting);
        */

        // Wait for the start of the game
        waitForStart();
        /*
        rotateLeftOrRight(-25, 0.25);

        //shooter
        maxShooterSpeed = -1.0;
        shooterSpeed = maxShooterSpeed;
            
        //if (shooterSpeed <= maxShooterSpeed) {
            shooterSpeed = maxShooterSpeed;
        //}
            
        shooter.setPower(shooterSpeed);
        
        sleep(1000);
            
        //if (shooterSpeed <= maxShooterSpeed) {
            collector.setPower(1.0);
        //}
        
        sleep(3000);
        
        collector.setPower(0.0);
        shooter.setPower(0.0);
        
        rotateLeftOrRight(15, 0.25);
        */
       
        moveForwardOrBack((-7 * 12), 0.15);
        
        // strafeLeftOrRight(12, 0.5);
        
        moveForwardOrBack(42, 0.25);
        
        strafeLeftOrRight(-16, 0.25);
        
        rotateLeftOrRight(-11, 0.25);
      
        // rotateLeftOrRight(-270, 0.5);
        //collector

        //shooter
        maxShooterSpeed = -0.70;
        shooterSpeed = maxShooterSpeed;
            
        //if (shooterSpeed <= maxShooterSpeed) {
            shooterSpeed = maxShooterSpeed;
        //}
            
        shooter.setPower(shooterSpeed);
        
        sleep(1000);
            
        //if (shooterSpeed <= maxShooterSpeed) {
            collector.setPower(1.0);
        //}
        
        sleep(3000);
        
        collector.setPower(0.0);
        shooter.setPower(0.0);
       
        moveForwardOrBack(-10, 0.25);

        // End Changes
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
            telemetry.addData("Target", "%7d :%7d", frontLeftPosition, frontRightPosition, backLeftPosition, backRightPosition);
            telemetry.addData("Actual", "%7d :%7d", frontLeftWheel.getCurrentPosition(),
                    frontRightWheel.getCurrentPosition(), backLeftWheel.getCurrentPosition(),
                    backRightWheel.getCurrentPosition());
            telemetry.update();
            idle();
        }

        frontLeftWheel.setPower(stopPower);
        backLeftWheel.setPower(stopPower);
        frontRightWheel.setPower(stopPower);
        backRightWheel.setPower(stopPower);
        
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
            telemetry.addData("Target", "%7d :%7d", frontLeftPosition, frontRightPosition, backLeftPosition, backRightPosition);
            telemetry.addData("Actual", "%7d :%7d", frontLeftWheel.getCurrentPosition(),
                    frontRightWheel.getCurrentPosition(), backLeftWheel.getCurrentPosition(),
                    backRightWheel.getCurrentPosition());
            telemetry.update();
            idle();
        }

        frontLeftWheel.setPower(stopPower);
        backLeftWheel.setPower(stopPower);
        frontRightWheel.setPower(stopPower);
        backRightWheel.setPower(stopPower);
        
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
            telemetry.addData("Target", "%7d :%7d", frontLeftPosition, frontRightPosition, backLeftPosition, backRightPosition);
            telemetry.addData("Actual", "%7d :%7d", frontLeftWheel.getCurrentPosition(),
                    frontRightWheel.getCurrentPosition(), backLeftWheel.getCurrentPosition(),
                    backRightWheel.getCurrentPosition());
            telemetry.update();
            idle();
        }

        frontLeftWheel.setPower(stopPower);
        backLeftWheel.setPower(stopPower);
        frontRightWheel.setPower(stopPower);
        backRightWheel.setPower(stopPower);
        
        encoderSetting = stopAndReset;
        frontLeftWheel.setMode(encoderSetting);
        backLeftWheel.setMode(encoderSetting);
        frontRightWheel.setMode(encoderSetting);
        backRightWheel.setMode(encoderSetting);

    }
}