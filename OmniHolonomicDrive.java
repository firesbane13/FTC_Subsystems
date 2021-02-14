package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This code was based on https://github.com/nahn20/FGC_Guides/blob/master/driveJava.java
 * as described in paper: https://seas.yale.edu/sites/default/files/imce/other/HolonomicOmniWheelDrive.pdf
 * It's been modifed and documented to fit the needs to help teach our students and
 * used in our robot
 */
@TeleOp(name="omniHolonomicDrive", group="2021UltimateGoal")
public class OmniHolonomicDrive extends LinearOpMode {

    float rotateAngle = 0;
    double resetAngle = 0;

    private DcMotor frontLeftWheel  = null;
    private DcMotor backLeftWheel   = null;
    private DcMotor backRightWheel  = null;
    private DcMotor frontRightWheel = null;

    BNO055IMU imu;
    @Override
    public void runOpMode() {
        frontLeftWheel  = hardwareMap.dcMotor.get("frontLeftWheel");
        backLeftWheel   = hardwareMap.dcMotor.get("backLeftWheel");
        backRightWheel  = hardwareMap.dcMotor.get("backRightWheel");
        frontRightWheel = hardwareMap.dcMotor.get("frontRightWheel");

        frontLeftWheel.setDirection(DcMotor.Direction.REVERSE);
        backLeftWheel.setDirection(DcMotor.Direction.REVERSE);
        frontRightWheel.setDirection(DcMotor.Direction.FORWARD);
        backRightWheel.setDirection(DcMotor.Direction.FORWARD);

        frontLeftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode             = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit        = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit        = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled   = false;

        imu.initialize(parameters);

        while(!opModeIsActive()){}

        while(opModeIsActive()){
            drive();
            resetAngle();
            telemetry.update();
        }
    }
    
    public void drive() {
        double protate = gamepad1.right_stickX/4;

        //Accounts for protate when limiting magnitude to be less than 1
        double stickX = gamepad1.left_stickX * Math.sqrt(Math.pow(1-Math.abs(protate), 2)/2); 
        double stickY = gamepad1.left_stickY * Math.sqrt(Math.pow(1-Math.abs(protate), 2)/2);
        
        double theta = 0;
        double pX    = 0;
        double pY    = 0;

        // 1/4 of the circle or 90 deg sections.
        // NOTE: I think this is in relation to the 4 quadrants
        double halfPi = Math.PI / 2;

        // Converts gyroAngle into radians
        double gyroAngle = getHeading() * Math.PI / 180; 
        
        if ( gyroAngle <= 0 ) {
            gyroAngle = gyroAngle + halfPi;
        } else if ( 0 < gyroAngle && gyroAngle < halfPi ) {
            gyroAngle = gyroAngle + halfPi;
        } else if ( halfPi <= gyroAngle ) {
            gyroAngle = gyroAngle - (3 * halfPi);
        }
        
        gyroAngle = -1 * gyroAngle;

        //Disables gyro, sets to -Math.PI/2 so front is defined correctly.
        if ( gamepad1.right_bumper ) { 
            gyroAngle = -halfPi;
        }

        //Linear directions in case you want to do straight lines.
        if ( gamepad1.dpad_right ) {
            stickX = 0.5;
        } else if ( gamepad1.dpad_left ) {
            stickX = -0.5;
        }

        if ( gamepad1.dpad_up ) {
            stickY = -0.5;
        } else if ( gamepad1.dpad_down ) {
            stickY = 0.5;
        }

        //MOVEMENT
        /**
         * The range of motion on a joystick is as follows:
         * 
         *              1.0
         *               |
         *        -1.0 --+-- 1.0
         *               |
         *             -1.0
         * 
         * Math.atan2 (inverse tangent) calculates the angle at the center of the circle from the x plane to the point
         * defined by the joystick's x and y coordinates.
         * 
         * gryoAngle is the angle of the gyroscope from the starting angle of the gyroscope to it's current position
         * 
         * halfPi is 90 degs or possibly splitting the circle into quadrants
         * 
         * theta calculates angle of the joysticks adjusted by the position(angle) of the gyroscope in relation to
         * 1/4 of the circle (90 deg).   This angle is used to help determine the power values for left/right and forward/back.
         * 
         * Math.sin finds the length of the side in relation to the angle from the center of the circle increased/reduced by
         * 1/8 of the circle or 45 deg sections.
         * 
         * NOTE: I think the use of the 45 deg angles is in relation to angles of the wheels on the robot in relation to what
         * the user would consider the robot's forward, back, left, and right sides.   So I think the Math.sin is adjusting the
         * position of the coordinates on the joystick/robot with the angle of the wheels on the robot.
         * 
         * x^2 + y^2 * sine(calculated center angle +/- the angle of the wheels) = power
         * 
         * You can apply the same power to wheels facing the same direction.
         * 
         *     pY /---\ pX
         *        |   |
         *     pX \---/ pY
         */
        theta = Math.atan2(stickY, stickX) - gyroAngle - halfPi;
        pX = Math.sqrt(Math.pow(stickX, 2) + Math.pow(stickY, 2)) * (Math.sin(theta + Math.PI / 4));
        pY = Math.sqrt(Math.pow(stickX, 2) + Math.pow(stickY, 2)) * (Math.sin(theta - Math.PI / 4));

        telemetry.addData("stickX", stickX);
        telemetry.addData("stickY", stickY);
        telemetry.addData("Magnitude",  Math.sqrt(Math.pow(stickX, 2) + Math.pow(stickY, 2)));
        telemetry.addData("Front Left", pY - protate);
        telemetry.addData("Back Left", pX - protate);
        telemetry.addData("Back Right", pY + protate);
        telemetry.addData("Front Right", pX + protate);

        frontLeftWheel.setPower(pY - protate);
        backLeftWheel.setPower(pX - protate);
        backRightWheel.setPower(pY + protate);
        frontRightWheel.setPower(pX + protate);
    }
    public void resetAngle() {
        if ( gamepad1.a ) {
            reset_angle = getHeading() + reset_angle;
        }
    }

    public double getHeading(){
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double heading = angles.firstAngle;

        if ( heading < -180 ) {
            heading = heading + 360;
        } else if ( heading > 180 ) {
            heading = heading - 360;
        }

        heading = heading - reset_angle;
        
        return heading;
    }
}
