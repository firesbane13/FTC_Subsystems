package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name="TestMecanum", group="2021UltimateGoal")
public class MecanumDriveWithGyroscope extends LinearOpMode {

    /**
     * this code is based on https://github.com/nahn20/FGC_Guides/blob/master/driveJava.java
     * as described in paper: https://seas.yale.edu/sites/default/files/imce/other/HolonomicOmniWheelDrive.pdf
     * It has been modified and documented to help teach our students
     *
     * Thing to note that despite the paper showing omniwheels in various orientations, it
     * only seems to work correctly for Mecanum wheels and not Omniwheels in a 45 degree angle.
     *
     * Mecanum wheels form an X with their rolls, but the Omniwheels in a 45 degree orientation
     * move differently for left, right, and diagonals.   In order to adjust this for Omniwheels
     * in a 45 degree orientation, the pX and pY values needed to be switched around in the final
     * wheel motor setPower() function calls.
     */

    /**
     * Setting constants to variables like this allows one to more easily change
     * configurations without massive changes.
     *
     * Example:
     *      Original code setDirection for 2 wheels as so:
     *      frontLeftWheel.setDirection(DcMotor.Direction.FORWARD);
     *      backLeftWheel.setDirection(DcMotor.Direction.FORWARD);
     *
     *      If we wanted to change directions, we'd have to change both of those
     *      from FORWARD to REVERSE.
     *
     *      By setting a leftDirection and rightDirection we can consistently set
     *      those directions here vs changing the motors further down.
     *
     *      New code:
     *      frontLeftWheel.setDirection(leftDirection);
     *      backLeftWheel.setDirection(leftDirection);
     */
    private DcMotor.Direction forward = DcMotor.Direction.FORWARD;
    private DcMotor.Direction reverse = DcMotor.Direction.REVERSE;
    private DcMotor.Direction leftDirection = forward;
    private DcMotor.Direction rightDirection = reverse;

    /**
     * Other ZeroPowerBehaviors can be added here as needed as well as changing the default
     * behavior.
     */
    private DcMotor.ZeroPowerBehavior brakeMode = DcMotor.ZeroPowerBehavior.BRAKE;
    private DcMotor.ZeroPowerBehavior zeroPowerBehavior = brakeMode;

    // Gyroscope: This is used to reset the gyroscope to a 0 heading
    double resetAngle = 0;

    // DcMotors: These are the code representation of the motors
    private DcMotor frontLeftWheel = null;
    private DcMotor backLeftWheel = null;
    private DcMotor frontRightWheel = null;
    private DcMotor backRightWheel = null;

    /**
     * Wheel orientation
     * 0 - Mecanum Wheels with rollers in X formation
     * 1 - Omniwheels at 45 degs / Mecanum in a box formation
     *
     * Since there's not much different between Mecanum and Omniwheels this was coded
     * so we could use this on multiple bots by switching the value of wheelOrientation.
     */
    private int wheelOrientation = 0;

    /*
     * IMU: This contains 3 different sensors in FTC on the Control Hub/Expansian Hub
     *
     * - Accelerameter: Used to determine the acceleration of the robot
     * - Gyroscope: Used to determine the robot's orientation from it's start position (0 deg is
     *              the initial start position when the robot is first turned on)
     * - Magnetometer: Used to measure magnetic forces, especially earth's magnetism.
     */
    BNO055IMU imu;

    // @Override is used to override the existing runOpMode() function in LinearOpMode
    @Override
    public void runOpMode() {
        /*
         * Used to connect the hardware wheel confirmation to the software wheel variables.
         *
         * hardwareMap is a variable defined in LinearOpMode which is why it can be used without
         * defining it in this class.    It is used to retrieve the configuration for the
         * requested value from the Control Hub/Expansion Hub.
         *
         * This code was written with the following ports in mind for each wheel
         * 0 = frontLeftWheel
         * 1 = backLeftWheel
         * 2 = frontRightWheel
         * 3 = backRightWheel
         */
        frontLeftWheel = hardwareMap.dcMotor.get("frontLeftWheel");
        backLeftWheel = hardwareMap.dcMotor.get("backLeftWheel");
        frontRightWheel = hardwareMap.dcMotor.get("frontRightWheel");
        backRightWheel = hardwareMap.dcMotor.get("backRightWheel");

        /*
         * By default all wheels are set to forward direction.   Above is defined the
         * leftDirection and rightDirection variables.   This is used to make sure the
         * left wheels are moving in the same direction and the right wheels are moving
         * in the same direction.
         */
        frontLeftWheel.setDirection(leftDirection);
        backLeftWheel.setDirection(leftDirection);
        frontRightWheel.setDirection(rightDirection);
        backRightWheel.setDirection(rightDirection);

        /*
         * Used as the default state when no power is being supplied to the motors from
         * the user.
         */
        frontLeftWheel.setZeroPowerBehavior(zeroPowerBehavior);
        backLeftWheel.setZeroPowerBehavior(zeroPowerBehavior);
        frontRightWheel.setZeroPowerBehavior(zeroPowerBehavior);
        backRightWheel.setZeroPowerBehavior(zeroPowerBehavior);

        /*
         * IMU: This is retrieving the IC2 configuration from the Control Hub/Expansion Hub
         * for the three sensors (accelerometer, gyroscope, and magnetometer).
         */
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        /*
         * IMU: Used to configure Gyroscope and Accelerometer
         */
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode             = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit        = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit        = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled   = false;

        // IMU: Initialize the IMU sensors with the desired settings.
        imu.initialize(parameters);

        // Waits until the user presses Play on the Driver Station.
        waitForStart();

        // While the TeleOp or other op mode is running.
        while(opModeIsActive()){
            /*
             * Run the drive function which checks what controls the user has used
             * and any defined sensors.
             */
            drive();

            // IMU: Check if the user has reset the starting angle of the Gyroscope
            resetAngle();

            // Logging: Update the Driver Station with any telemetry logging done in the above functions.
            telemetry.update();
        }
    }

    public void drive(){
        /*
         * protate is used to adjust the speed of the robot's rotation.
         *
         * This can be divided by any number but we chose 4 as it was the smoothest for rotation.
         *
         * The range of this when divided by 4 is -0.25 <= protate <= 0.25 radians
         */
        double protate = gamepad1.right_stick_x / 4;

        /*
         * The total power cap should be 1, as that's the limitation of the DcMotor.  The
         * power cap of linear motion is 1 - protate.  The maximum magnitude will happen when
         * stick_x = stick_y = 1
         *
         * This formula is based on the pythagorean theorem (a^2 + b^2 = c^2).
         *
         * In this case, we know that the c^2, the hypotenuse, is c^2 = 1 - protate
         *
         * Since we know the maximum magnitude of stick_x = stick_y = 1, we can change the formula to
         *
         * 1 - protate = Math.sqrt(stick_x^2 + stick_y^2) = Math.sqrt(2 * stick_x^2) = Math.sqrt(2 * stick_y^2)
         *
         * Solve for stick_x.
         * NOTE: We could have used stick_y, but since we are calculating magnitude and they both
         * have a meximum of 1, thus being equal, it doesn't really matter.  So, I chose stick_x.
         *
         * 1 - protate = Math.sqrt(2 * stick_x^2)
         *
         * (1 - protate)^2 = 2 * stick_x^2
         *
         * (1 - protate)^2 / 2 = stick_x^2
         *
         * Math.sqrt((1 - protate)^2 / 2) = stick_x
         *      OR
         * stick_x = Math.sqrt((1 - protate)^2 / 2)
         *
         * The range of this is 0.5303 <= processed protate <= 0.707 radians
         *
         * This value relates to the turn.
         * If controller at 0, then power turn adjustment is 0.5303 radians.
         * If controller at 1, then power turn adjustment is 0.707 radians.
         */
        double processedProtate = Math.sqrt( Math.pow( 1 - Math.abs( protate ), 2 ) / 2 );

        /*
         * Analog stick * (magnitude value between 0.5303 - 0.707)
         */
        double stickX = gamepad1.left_stick_x * processedProtate;
        double stickY = gamepad1.left_stick_y * processedProtate;

        // Is the angle of the left analog stick in radians.
        double theta    = 0;

        // Is the magnitude/hypotenuse of the analog stick.
        double cValue   = 0;

        // Calculated magnitude to be sent to the DcMotor.
        double pX       = 0;
        double pY       = 0;

        // 1/2 PI = 1.5707 radians = 90 degrees
        double halfPi = Math.PI / 2;

        // 1/4 PI = 0.7853 radians = 45 degrees
        double quarterPi = Math.PI / 4;

        /*
         * Converts Gyroscope angle (degrees) into radians
         *
         * 1 radian = Pi / 180 = 0.01745
         *
         * getHeading() converts 360 to -180 to 180.   The reason for this is to denote if the robot
         * is turned left (-1 to -180 deg) or right (1 to 180 deg).
         *
         * gyroAngle = (left or right degrees) * (1 radian = 0.0.1745)
         *
         * Range -3.141 radians (-180 deg) <= gyroAngle <= 3.141 radians (180 deg)
         *
         * I'm assuming we're rotating the final angle halfPi for the same reason we will rotate it
         * in theta.   The axis for the controller is:
         *
         *        -1
         *         |
         *   -1 <--+--> 1
         *         |
         *         1
         *
         * and we want the Cartesian plane to be the correct orientation.
         */
        double gyroAngle = getHeading() * Math.PI / 180;

        if ( gyroAngle <= 0 ) {
            // if gyroAngle between 0.000 to -3.141 radians

            // gyroAngle = (0.000 to -3.141) + 1.5707
            // Range -1.5707 to 1.5707
            gyroAngle = gyroAngle + halfPi;
        } else if ( 0 < gyroAngle  && gyroAngle < halfPi ) {
            // if 0 radians < gyroAngle < 1.5707 radians

            // gyroAngle = (0.001 to 1.5706 radians) + 1.5707 radians
            // Range 1.5708 to 3.141 radians
            gyroAngle = gyroAngle + halfPi;
        } else if ( halfPi <= gyroAngle ) {
            // if 1.5707 radians <= gyroAngle (1.5707 to 3.141 radians

            // gyroAngle = (1.5707 to 3.141) - (3 * 1.5707)
            // gyroAngle = (1.5707 to 3.141) - 4.7121

            // Range -3.141 to -1.5707
            gyroAngle = gyroAngle - (3 * halfPi);
        }

        gyroAngle = -1 * gyroAngle;

        //Disables gyro, sets to -Math.PI/2 so front is defined correctly
        if( gamepad1.right_bumper ) {
            gyroAngle = -halfPi;
        }

        // Fixed power linear directions in case you want to do straight lines
        if (gamepad1.dpad_right) {
            stickX = 0.5;
        } else if ( gamepad1.dpad_left ){
            stickX = -0.5;
        }

        if (gamepad1.dpad_up) {
            stickY = 0.5;
        } else if ( gamepad1.dpad_down ) {
            stickY = -0.5;
        }

        //MOVEMENT
        /**
         * the range of motion on a joystick:
         *       -1.0
         *        |
         *  -1.0 -+- 1.0`
         *        |
         *       1.0
         * using an inverse tangent you can calculate the angle at the center is the circle from the X plane to the point
         * defined by the joystick's X and Y coordinates
         *
         * gyroangle is the angle the gyroscope is from the starting angle to its current position
         *
         * halfPi is 90 degrees or splitting the circle into quadrants
         *
         * theta calculates the angle of the joysticks adjusted by the position(angle) of the gyroscope in relation to
         * 1/4 of the circle (90 degrees). this angle is used to help determine the power values for left/right and forward/back
         *
         * Math.sin finds the length of the side in relation to the angle from the center of the circle increased/reduced by
         * 1/8 of the circle or 45 degree sections
         * NOTE: Math.sin is most likely adjusting the position of the coordinates on the joystick/robot with the angle of the wheels
         *
         * x^2 + y^2 * sine(calculated center angle +/- the angle of the wheels) = power
         *
         *    Omniwheels 45 deg         Mecanum in X
         *      pX /---\ pY             pY \---/ pX
         *         |   |                    | |
         *      pY \---/ pX             pX /---\ pY
         *
         *  halfPi = 1.5707
         *  gyroAngle =
         */
        theta = Math.atan2(stickY, stickX) - gyroAngle - halfPi;

        /**
         * this formula is calculating the value of c in the Pythagorean theorem, but using
         * stickX for 'a' and stickY for 'b'
         *
         * C being the length of the radial line from the center
         *
         * then to take the radial line and rotate based on the calculated amount based on calculated
         * angle and the angle of the robot
         */
        cValue = Math.sqrt(Math.pow(stickX, 2) + Math.pow(stickY, 2));

        // We're rotating values by 45 deg because of the angle of the wheels
        pX = cValue * (Math.sin(theta + quarterPi));
        pY = cValue * (Math.sin(theta - quarterPi));

        telemetry.addData("stickX", stickX);
        telemetry.addData("stickY", stickY);
        telemetry.addData("Magnitude", cValue);

        if ( wheelOrientation == 0 ) {
            // Mecanum wheels in an X formation
            telemetry.addData("Front Left", pY - protate);
            telemetry.addData("Back Left", pX - protate);
            telemetry.addData("Back Right", pY + protate);
            telemetry.addData("Front Right", pX + protate);

            // Mecanum
            frontLeftWheel.setPower(pY - protate);
            backLeftWheel.setPower(pX - protate);
            frontRightWheel.setPower(pX + protate);
            backRightWheel.setPower(pY + protate);
        } else if ( wheelOrientation == 1 ) {
            // Omniwheels in a 45 deg formation
            telemetry.addData("Front Left", pX - protate);
            telemetry.addData("Back Left", pY - protate);
            telemetry.addData("Back Right", pX + protate);
            telemetry.addData("Front Right", pY + protate);

            // Omniwheels 45 deg
            frontLeftWheel.setPower(pX - protate);
            backLeftWheel.setPower(pY - protate);
            frontRightWheel.setPower(pY + protate);
            backRightWheel.setPower(pX + protate);
        }
    }

    /**
     * Used to set the gyroscope starting angle.
     * the initial starting angle is based on when the robot is turned on
     * if the robot is moved after it is on, it needs to have a starting position
     *
     * EXTRA: maybe this can be automatically
     */
    public void resetAngle(){
        if (gamepad1.a){
            resetAngle = getHeading() + resetAngle;
        }
    }

    public double getHeading(){
        // Axesreference.INTRINSIC = intrinsic rotations, where the axes move with the object that is rotating.
        // AxesOrder.ZYX = the order of the axes are returned.
        // AngleUnit.DEGREES = Returns the angle in DEGREES.
        Orientation angles = imu.getAngularOrientation(
                AxesReference.INTRINSIC,
                AxesOrder.ZYX,
                AngleUnit.DEGREES
        );

        /*
         * Retrieves the first axis' value in degrees
         */
        double heading = angles.firstAngle;
        if ( heading < -180 ) {
            heading = heading + 360;
        } else if ( heading > 180 ) {
            heading = heading - 360;
        }

        heading = heading - resetAngle;

        return heading;
    }
}

