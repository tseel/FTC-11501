package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

public class MecanumRobot {
    
    private ElapsedTime runtime = new ElapsedTime();
    
    // when initializing, stores the current opMode
    private LinearOpMode opMode;
    
    // constants for the marker servo
    public static final double markerOpen = (double) 85 / 180;
    public static final double markerClose = 1;

    // Motor and servo variables
    public DcMotor frontRight;
    public DcMotor frontLeft;
    public DcMotor backLeft;
    public DcMotor backRight;
    public Servo markerServo;
    public DcMotor liftUp;
    public DcMotor liftDown;

    // Variables for imu 
    private BNO055IMU imu;
    private Orientation angles;
    private Acceleration gravity;
    
    // Encoder numbers
    private final double WHEEL_CIRCUMFERENCE = 4 * Math.PI;
    private final double GEAR_RATIO = 1;
    private final int TICKS_PER_REV = 1120; 
    private final double DRIVE_SPEED = 0.75;
    
    private final double LIFT_GEAR_RATIO = (double) 80 / 40;
    private final double LIFT_SPEED = 0.5; 
    
    /**
     * @param opMode the OpMode to be used with the setup
     */
    public MecanumRobot(LinearOpMode opMode) {
        this.opMode = opMode;
    }
    
    /**
     * Drive method for Autonomous
     * @param dist distance in inches to drive
     */
    public void autoDrive(double dist) {
        // set encoders to
        int counts = (int) Math.round(((dist / WHEEL_CIRCUMFERENCE) / GEAR_RATIO) * TICKS_PER_REV);
        
        // reset all encoders
        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        setMotorPositions(counts);
        
        setMode(DcMotor.RunMode.RUN_TO_POSITION);

        setPower(DRIVE_SPEED);
        
        while (isMotorsBusy() && opMode.opModeIsActive()) {
            encoderTel();
        }
        opMode.telemetry.update();
        setPower(0);

        setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    
    /**
     * Drive method for TeleOp
     * @param drive value -1 (backward) to 1 (forward) on how much to drive forward/backward
     * @param strafe value -1 (left) to 1 (right) on how much to strafe
     * @param rotate value -1 (left) to 1 (right) on how much to rotate
     */
    public void manualDrive(double drive, double strafe, double rotate) {
        double frontRPower = drive + strafe - rotate;
        double frontLPower = drive - strafe + rotate;
        double backRPower  = drive - strafe - rotate;
        double backLPower  = drive + strafe + rotate;
            
        frontRight.setPower(frontRPower);
        frontLeft.setPower(frontLPower);
        backRight.setPower(backRPower);
        backLeft.setPower(backLPower);
        
        // TODO: add telemetry
    }
    
    /**
     * method to turn in Autonomous
     * @param degrees distance in degrees to turn
     * @param dir direction to turn using a Direction enum
     */
    public void turn(double degrees, Direction dir) {
        // constants used for both directions
        double TURNING_SPEED = 0.75;
        double SLOWDOWN_DISTANCE = 0.6; // fraction of distance before bot slows down
            
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double startAngle = normAngle(angles.firstAngle);
        if (dir == Direction.RIGHT) {
            double target = startAngle - degrees;
            double slowdown = startAngle - SLOWDOWN_DISTANCE * degrees;
            // Correct values by adding 360 if they are under 0
            target = target <= 0 ? target + 360 : target;
            slowdown = slowdown <= 0 ? slowdown + 360 : slowdown;
            turnLoop(-TURNING_SPEED, target, slowdown);
        } else if (dir == Direction.LEFT) {
            double target = startAngle + degrees;
            double slowdown = startAngle + SLOWDOWN_DISTANCE * degrees;
            // Correct values by subtracting 360 if they are over 360
            target = target >= 360 ? target - 360 : target;
            slowdown = slowdown >= 360 ? slowdown - 360 : slowdown;
            turnLoop(TURNING_SPEED, target, slowdown);
        }
    }
    /**
     * loop for Autonomous turning
     * @param speed amount of power to give motors
     * @param target the target heading to end at
     * @param slowdown fraction of distance (between 0 and 1) before bot slows down
     */
    private void turnLoop(double speed, double target, double slowdown) {
        double TURNING_PRECISION = 2; // amount of degrees that allows stopping
        boolean turn = true; // dummy variable because I hate do-while loops
        while (turn) {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            double curAngle = normAngle(angles.firstAngle);
            if (Math.abs(curAngle - slowdown) <= TURNING_PRECISION) speed *= 0.8;
                frontLeft.setPower(speed);
                backLeft.setPower(speed);
                frontRight.setPower(-speed);
                backRight.setPower(-speed);
            if (Math.abs(curAngle - target) <= TURNING_PRECISION || !opMode.opModeIsActive()) turn = false;
        }
        setPower(0); // stop bot
    }

    /**
     * method to strafe in Autonomous
     * @param degrees distance in inches to strafe
     * @param dir direction to strafe using a Direction enum
     */
    public void strafe(double dist, Direction direction) {
        // i don't know why this is multiplied by 1.5 but it seems to be pretty accurate
        int counts = (int) Math.round(((dist / WHEEL_CIRCUMFERENCE) / GEAR_RATIO) * TICKS_PER_REV * 1.5);
        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        // TODO: refactor this? might not be worth it
        if (direction == Direction.RIGHT) {
            backLeft.setTargetPosition(-counts);
            frontLeft.setTargetPosition(counts);
            backRight.setTargetPosition(counts);
            frontRight.setTargetPosition(-counts);
        } else if (direction == Direction.LEFT) {
            backLeft.setTargetPosition(counts);
            frontLeft.setTargetPosition(-counts);
            backRight.setTargetPosition(-counts);
            frontRight.setTargetPosition(counts);
        }
        
        setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        setPower(DRIVE_SPEED);
        
        while (isMotorsBusy() && opMode.opModeIsActive()) {
            encoderTel();
        }
        opMode.telemetry.update();
        setPower(0);

        setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    
    /**
     * method to drop the marker and then sleep
     */
    public void dropMarker() {
        markerServo.setPosition(markerOpen);
        opMode.sleep(500);
    }
    
    /** 
     * method to close the marker servo
     */
    public void closeMarkerServo() {
        markerServo.setPosition(markerClose);
    }
    
    /**
     * method to operate the lift
     * @param dist distance (in inches) to move the lift. negative numbers go down.
     */
     public void moveLift(double dist) {
        int counts = (int) Math.round((dist / LIFT_GEAR_RATIO) * TICKS_PER_REV * 4.0/3.0);
        liftUp.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftDown.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        liftUp.setTargetPosition((int) Math.round(counts*1.125));
        liftDown.setTargetPosition(-counts);
        
        liftUp.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftDown.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        liftUp.setPower(LIFT_SPEED);
        liftDown.setPower(LIFT_SPEED);
        
        while (isLiftBusy() && opMode.opModeIsActive()) {
        
            
        }

        liftUp.setPower(0);
        liftDown.setPower(0);

        liftUp.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftDown.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
     }
    
    /**
     * Initializes all motors and servos and sets direction. Run at start of any OpMode.
     */
    public void initDrive() {
        
        // initialize hardware
        frontRight  = opMode.hardwareMap.get(DcMotor.class, "front_right");
        frontLeft   = opMode.hardwareMap.get(DcMotor.class, "front_left");
        backRight   = opMode.hardwareMap.get(DcMotor.class, "back_right");
        backLeft    = opMode.hardwareMap.get(DcMotor.class, "back_left");
        markerServo = opMode.hardwareMap.get(Servo.class, "marker_servo");
        liftUp      = opMode.hardwareMap.get(DcMotor.class, "lift_up");
        liftDown    = opMode.hardwareMap.get(DcMotor.class, "lift_down");

        // set motor directions
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        liftUp.setDirection(DcMotor.Direction.FORWARD);
        liftDown.setDirection(DcMotor.Direction.REVERSE);
        
        liftUp.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftDown.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // set motors runmodes
        setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftUp.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftDown.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    
    // unused
    // TODO: set this up
    /**
     * initializes Vuforia. Use at the start of autonomous opmodes using Vuforia | UNFINISHED
     */
    public void initVuforia() {
        VuforiaLocalizer vuforia;
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        // vuforia setup
        //parameters.vuforiaLicenseKey = "license key here";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK; // use back camera
        //vuforia = ClassFactory.createVuforiaLocalizer(parameters); // create the vuforia localizer
        //VuforiaTrackables roverTrackables = vuforia.loadTrackablesFromAsset(""); // TODO: find the name for the trackables
    }
    
    /**
     * initializes the IMU (gyroscope)
     */
    public void initImu() {
        BNO055IMU.Parameters imuParameters = new BNO055IMU.Parameters();
        imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES; // wouldn't return radians even when I set this to radians?
        imuParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imuParameters.mode = BNO055IMU.SensorMode.IMU;
        imuParameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // i honestly have no idea if this file was created correctly since I'm using OnBotJava
        imuParameters.loggingEnabled = true;
        imuParameters.loggingTag = "IMU";
        imu = opMode.hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(imuParameters);
    }
    
    /**
     * simple way to set the mode of all drive motors at once
     * @param mode the mode to set the motors to
     */
    private void setMode(DcMotor.RunMode mode) {
        frontRight.setMode(mode);
        frontLeft.setMode(mode);
        backRight.setMode(mode);
        backLeft.setMode(mode);
    }
    
    /**
     * simple way to set the power of all drive motors equal. usually used to stop the bot
     * @param power a decimal -1 to 1 of the power to set the motors to
     */
    private void setPower(double power) {
        frontRight.setPower(power);
        frontLeft.setPower(power);
        backRight.setPower(power);
        backLeft.setPower(power);
    }
    
    /**
     * checks if any drive motor is currently busy
     * @return true if any drive motor is busy
     */
    public boolean isMotorsBusy() {
        return backLeft.isBusy() || backRight.isBusy() || frontLeft.isBusy() || frontRight.isBusy();
    }
    
    /**
     * checks if either lift motor is currently busy
     * @return true if any lift motor is busy
     */
     public boolean isLiftBusy() {
         return liftUp.isBusy() || liftDown.isBusy();
     }
    
    
    /**
     * simple function to allow setting all drive motors to the same position
     * @param pos position to set the motors to.
     */
    private void setMotorPositions(int pos) {
        backLeft.setTargetPosition(pos);
        backRight.setTargetPosition(pos);
        frontLeft.setTargetPosition(pos);
        frontRight.setTargetPosition(pos);
    }
    
    /**
     * normalizes angles to all be positive
     * adds 360 to an angle if it's under 0, used because the gyro returns negative angles between 180 and 360
     * @param angle angle to make positive
     */
    private double normAngle(double angle) {
        return angle > 0 ? angle : 360 + angle;
    }

    /**
     * 
     * telemetry for the encoders
     */
    private void encoderTel() {
        opMode.telemetry.addData("Front left", "%d/%d %.2f %s", frontLeft.getCurrentPosition(), frontLeft.getTargetPosition(), frontLeft.getPower(), frontLeft.isBusy() ? "Busy" : "Finished");
        opMode.telemetry.addData("Front right", "%d/%d %.2f %s", frontRight.getCurrentPosition(), frontRight.getTargetPosition(), frontRight.getPower(), frontRight.isBusy() ? "Busy" : "Finished");
        opMode.telemetry.addData("Back left", "%d/%d %.2f %s", backLeft.getCurrentPosition(), backLeft.getTargetPosition(), backLeft.getPower(), backLeft.isBusy() ? "Busy" : "Finished");
        opMode.telemetry.addData("Back right", "%d/%d %.2f %s", backRight.getCurrentPosition(), backRight.getTargetPosition(), backRight.getPower(), backRight.isBusy() ? "Busy" : "Finished");
        opMode.telemetry.update();
    }
}