package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name="PearlsMecanum", group="2021UltimateGoal")
public class PearlsMecanum extends LinearOpMode {

/**
 * this code is based on https://github.com/nahn20/FGC_Guides/blob/master/driveJava.java
 * as described in paper: https://seas.yale.edu/sites/default/files/imce/other/HolonomicOmniWheelDrive.pdf
 * It has been modifed and documented to help teach our students
 * and remodified my Julia to prove comprehension and fit current coding level
 */
 
    float rotateAngle = 0;
    double resetAngle = 0;
    double shooterSpeed;
    double maxShooterSpeed;
    
    //drive train
    private DcMotor frontLeftWheel = null;
    private DcMotor backLeftWheel = null;
    private DcMotor frontRightWheel = null;
    private DcMotor backRightWheel = null;
    
    //shooter and collector
    private DcMotor collector;
    private DcMotor shooter;
    
    //arm and claw
    private DcMotor arm;
    private Servo claw;
    
    private DcMotor.RunMode encoderSetting;
    private DcMotor.RunMode stopAndReset = DcMotor.RunMode.STOP_AND_RESET_ENCODER;
    private DcMotor.RunMode runToPosition = DcMotor.RunMode.RUN_TO_POSITION;
    private DcMotor.RunMode runUsingEncoder = DcMotor.RunMode.RUN_USING_ENCODER;

    static final double stopPower = 0.0;
    static final int cpr = 960;
    double armRadius = 12.5;
    double movementPerDegree = (armRadius * cpr) / 360;
    
    BNO055IMU imu;
    @Override
    public void runOpMode() {
        frontLeftWheel = hardwareMap.dcMotor.get("frontLeftWheel");
        backLeftWheel = hardwareMap.dcMotor.get("backLeftWheel");
        frontRightWheel = hardwareMap.dcMotor.get("frontRightWheel");
        backRightWheel = hardwareMap.dcMotor.get("backRightWheel");
        
        collector = hardwareMap.dcMotor.get("collector");
        shooter = hardwareMap.dcMotor.get("shooter");
        arm = hardwareMap.dcMotor.get("arm");
        claw = hardwareMap.servo.get("claw");
        
        frontLeftWheel.setDirection(DcMotor.Direction.FORWARD);
        backLeftWheel.setDirection(DcMotor.Direction.FORWARD);
        frontRightWheel.setDirection(DcMotor.Direction.REVERSE);
        backRightWheel.setDirection(DcMotor.Direction.REVERSE);
        
        
        frontLeftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        //the expansion hub has an accelerometer, gyroscope, and magnetometer built in. we will be focusing on the gyroscope.
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
            
            //left and right claw opposite degrees
            //keep left claw negative
            if (gamepad2.y) {
                claw.setPosition(1);
            } else if (gamepad2.x){
                claw.setPosition(0);
            }
            
            if (gamepad2.right_bumper) {
                // arm.setPower(1.0);
                rotateLeftOrRight(90, 0.1);
            } else if (!gamepad2.right_bumper){
                arm.setPower(0.0);
            }
            
            if (gamepad2.left_bumper) {
                //arm.setPower(-1.0);
                rotateLeftOrRight(-90, 0.1);
            } else if (!gamepad2.left_bumper) {
                arm.setPower(0.0);
            }
            
            //collector
            collector.setPower(gamepad1.left_trigger);
            
            //shooter
            maxShooterSpeed = -0.70;
            shooterSpeed = -gamepad1.right_trigger;
            
            if (shooterSpeed <= maxShooterSpeed) {
                shooterSpeed = maxShooterSpeed;
            }
            
            shooter.setPower(shooterSpeed);

            if (shooterSpeed <= maxShooterSpeed) {
               sleep(250);
            collector.setPower(1.0);
            
            }
            
        }
        
    }
    
    public void drive(){
        //this can be divided by any number but we chose 4
        double protate = gamepad1.right_stick_x / 4;
        
        //accounts for protate when limiting magnitude to be less than 1
        double processedProtate = Math.sqrt( Math.pow( 1 - Math.abs( protate ), 2 ) / 2 );
        double stickX = gamepad1.left_stick_x * processedProtate;
        double stickY = gamepad1.left_stick_y * processedProtate;
        
        double cValue   = 0;
        double theta    = 0;
        double pX       = 0;
        double pY       = 0;
        
        // 1/4 of the circle or 90 degree sections
        double halfPi = Math.PI / 2;
        
        // 1/8 of the circle or 45 degree sections
        double quarterPi = Math.PI / 4;
        
        //converts gyroAngle into radians
        double gyroAngle = getHeading() * Math.PI / 180;
        
        if ( gyroAngle <= 0 ) {
            gyroAngle = gyroAngle + halfPi;
        } else if ( 0 < gyroAngle  && gyroAngle < halfPi ) {
            gyroAngle = gyroAngle + halfPi;
        } else if ( halfPi <= gyroAngle ) {
            gyroAngle = gyroAngle - (3 * halfPi);
        }
        
        gyroAngle = -1 * gyroAngle;
        
        //Disables gyro, sets to -Math.PI/2 so front is defined correctly
        //if( gamepad1.right_bumper ) {
            gyroAngle = -halfPi;
        //}
        
        //linear directions in case you want to do straight lines
        if (gamepad1.dpad_right) {
            stickX = -0.5;
        } else if ( gamepad1.dpad_left ){
            stickX = -0.5;
        }
        
        if (gamepad1.dpad_up) {
            stickY = -0.5;
        } else if ( gamepad1.dpad_down ) {
            stickY =0.5;
        }
        
        //MOVEMENT
        /**
         * the range of motion on a joystick:
         *       1.0
         *        |
         *  -1.0 -+- 1.0
         *        |
         *      -1.0
         * using an inverse tangent you can calculate the angle at the center is the circle from the X plane to the point 
         * defined by the joystick's X and Y coordinates 
         * 
         * gyroangle is the angle the gyroscope from the starting angle to it's current position 
         * 
         * halfPi is 90 degrees or splitting the circle into quadrants 
         * 
         * theta calculates the angle of the joysticks adjusted by the position(angle) of the gyroscope in relation to
         * 1/4 of the circle (90 degrees). this angle is used to help determine the power values for left/right and foward/back 
         * 
         * Math.sin finds the length of the side in relation to the angle from the center of the circle increased/reduced by
         * 1/8 of the circle or 45 degree sections
         * NOTE: Math.sin is most likely adjusting the position of the coordinates on the joystick/robot with the angle of the wheels
         * 
         * x^2 + y^2 * sine(calculated center andgle +/- the angle of the wheels) = power
         * 
         *      pY /---\ pX
         *         \   \
         *      pX \---/ pY
         */
        theta = Math.atan2(stickY, stickX) - gyroAngle - halfPi;
        
        /**
         * this formula is calculating the value of c in the Pythagorean theorem, but using 
         * stickX for 'a' and stickY for 'b'
         * 
         * C being the lenth of the radial line from the center
         * 
         * then to take the radial line and rotate based on the calculated amount based on calculated 
         * angle and the angle of the robot 
         */
        cValue = Math.sqrt(Math.pow(stickX, 2) + Math.pow(stickY, 2));
        
        pX = cValue * (Math.sin(theta + quarterPi));
        pY = cValue * (Math.sin(theta - quarterPi));
        
        telemetry.addData("stickX", stickX);
        telemetry.addData("stickY", stickY);
        telemetry.addData("Magnitude", cValue);
        telemetry.addData("Front Left", pY + protate);
        telemetry.addData("Back Left", pX + protate);
        telemetry.addData("Back Right", pY - protate);
        telemetry.addData("Front Right", pX - protate);
        
        frontLeftWheel.setPower(pY - protate);
        backLeftWheel.setPower(pX - protate);
        frontRightWheel.setPower(pX + protate);
        backRightWheel.setPower(pY + protate);
    }
    /** 
     *  Used to set the gyroscope starting angle.
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
        // Axesreference.INTRINSIC = inrinsic rotations, where the axes move with the object that is rotating.
        // AxesOrder.ZYX = the order of the axes are returned.
        // AngleUnit.DEGREES = Returns the angle in DEGREES.
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        
        //Retrieves the first axis' value
        double heading = angles.firstAngle;
        
        if ( heading < -180 ) {
            heading = heading + 360;
        } else if ( heading > 180 ) {
            heading = heading - 360;
        }
        
        heading = heading - resetAngle;
        
        return heading;
        
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
        int armPosition = arm.getCurrentPosition();
        double movement = rotateAngle * movementPerDegree;

        // Apply movement
        armPosition += movement;

        // Set location to move to
        arm.setTargetPosition(armPosition);

        encoderSetting = runToPosition;
        arm.setMode(encoderSetting);
        
        // Start moving robot by setting motor speed
        arm.setPower(speed);

        while (arm.isBusy()
        ) {
            idle();
        }

        arm.setPower(stopPower);
        
        encoderSetting = stopAndReset;
        arm.setMode(encoderSetting);
    }
}