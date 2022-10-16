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
public class PearlsAutonomous extends LinearOpMode {

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

    Orientation             lastAngles = new Orientation();
    double                  globalAngle, power = .30, correction;

    // Tetrix Torquenado 44260, counts per revolution = 1440
    static final int cpr = 1440;

    static final int defaultInches = 1;
    static final int defaultDegree = 1;
    static final int wheelDiameter = 4;
    static final int robotRadius = 9;
    static final double armRadius = 12.5;
    static final double shaftRadius = 3;
    double movementInInches = cpr / (wheelDiameter * Math.PI);
    double robotPerDegree = ((robotRadius / (wheelDiameter / 2)) * cpr) / 360;
    double armPerDegree = ((armRadius / shaftRadius) * cpr) / 360;
    double distance = 0.0;

    static final double stopPower = 0.0;
    // End Change

    BNO055IMU imu;
    @Override
    public void runOpMode() {
        telemetry.setAutoClear(true);
        
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

        frontLeftWheel  = hardwareMap.dcMotor.get("frontLeftWheel");
        backLeftWheel   = hardwareMap.dcMotor.get("backLeftWheel");
        backRightWheel  = hardwareMap.dcMotor.get("backRightWheel");
        frontRightWheel = hardwareMap.dcMotor.get("frontRightWheel");

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

        // moveForwardOrBack(12, 0.5);
        
        // strafeLeftOrRight(12, 0.5);
        
        // moveForwardOrBack(-12, 0.5);
        
        // strafeLeftOrRight(-12, 0.5);
        
        rotateLeftOrRight(45, 0.25, robotPerDegree);
        
        rotateLeftOrRight(-270, 0.5, robotPerDegree);
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

            // Use gyro to drive in a straight line.
            correction = checkDirection();

            frontLeftWheel.setPower(speed - correction);
            backLeftWheel.setPower(speed - correction);
            frontRightWheel.setPower(speed + correction);
            backRightWheel.setPower(speed + correction);

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

        resetAngle();
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
            // Use gyro to drive in a straight line.
            correction = checkDirection();

            frontLeftWheel.setPower(speed - correction);
            backLeftWheel.setPower(speed - correction);
            frontRightWheel.setPower(speed + correction);
            backRightWheel.setPower(speed + correction);
            
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

        resetAngle();
    }

    /** 
     * rotateLeftOrRight()
     * 
     * Rotate angle left or right
     * 
     * @params int rotateAngle Postive value rotates right, negative rotates left
     * @params double speed Range 0.0 to 1.0
     */
    private void rotateLeftOrRight(int rotateAngle, double speed, double perDegree) {

        // Get current motor positions
        int frontLeftPosition = frontLeftWheel.getCurrentPosition();
        int backLeftPosition = backLeftWheel.getCurrentPosition();
        int frontRightPosition = frontRightWheel.getCurrentPosition();
        int backRightPosition = backRightWheel.getCurrentPosition();
        double movement = rotateAngle * perDegree;

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

        resetAngle();
    }

    /**
     * Resets the cumulative angle tracking to zero.
     */
    private void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right.
     */
    private double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    /**
     * See if we are moving in a straight line and if not return a power correction value.
     * @return Power adjustment, + is adjust left - is adjust right.
     */
    private double checkDirection()
    {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction, angle, gain = .10;

        angle = getAngle();

        if (angle == 0)
            correction = 0;             // no adjustment.
        else
            correction = -angle;        // reverse sign of angle for correction.

        correction = correction * gain;

        return correction;
    }
}