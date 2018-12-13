package org.firstinspires.ftc.TestCodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


@Autonomous(name = "Mecanum Autonomous", group = "Live")
public class MecanumAuto extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    // Motor and servo variables
    private DcMotor frontRight;
    private DcMotor frontLeft;
    private DcMotor backLeft;
    private DcMotor backRight;
    private Servo markerServo;
    
    // Variables for imu 
    private BNO055IMU imu;
    private Orientation angles;
    private Acceleration gravity;
    
    // Encoder numbers
    private final double WHEEL_CIRCUMFERENCE = 4 * Math.PI;
    private final double GEAR_RATIO = 1;
    private final int TICKS_PER_REV = 1120; 
    private final double DRIVE_SPEED = 0.75;
    private final double OFFSET = 6;

    
    final double markerClose = 0;
    final double markerOpen = (double) 115 / 180;

    VuforiaLocalizer vuforia;
    VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();


    @Override
    public void runOpMode() {
        /*
         * SETUP
         */
        
        // initialize hardware
        frontRight  = hardwareMap.get(DcMotor.class, "front_right");
        frontLeft   = hardwareMap.get(DcMotor.class, "front_left");
        backRight   = hardwareMap.get(DcMotor.class, "back_right");
        backLeft    = hardwareMap.get(DcMotor.class, "back_left");
        markerServo = hardwareMap.get(Servo.class, "marker_servo");
        
        // set motor directions
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        
        // set motors runmodes
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        setMotorsBrake();

        
        // vuforia setup
        //parameters.vuforiaLicenseKey = "license key here";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK; // use back camera
        //vuforia = ClassFactory.createVuforiaLocalizer(parameters); // create the vuforia localizer
        //VuforiaTrackables roverTrackables = vuforia.loadTrackablesFromAsset(""); // TODO: find the name for the trackables
        
        
        // imu setup
        BNO055IMU.Parameters imuParameters = new BNO055IMU.Parameters();
        imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imuParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imuParameters.calibrationDataFile = "AdafruitIMUCalibration.json";
        imuParameters.loggingEnabled = true;
        imuParameters.loggingTag = "IMU";
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(imuParameters);
        
        
        // wait for user to start opmode
        waitForStart();
        runtime.reset();
        
        //roverTrackables.activate();
        
        //driveDist(2 * 23.5 * Math.sqrt(2));
        //drive(0, 0.5, 0);
        //turn(45, Direction.LEFT);
        // driveDist(4.5 * 23.5);
        
        driveDist(12);
        //turn(45, Direction.LEFT);
    }
    
    private void drive(double drive, double strafe, double rotate) {
        double frontRPower = drive + strafe - rotate;
        double frontLPower = drive - strafe + rotate;
        double backRPower  = drive + strafe + rotate;
        double backLPower  = drive - strafe - rotate;
            
        frontRight.setPower(frontRPower);
        frontLeft.setPower(frontLPower);
        backRight.setPower(backRPower);
        backLeft.setPower(backLPower);
    }
    
    private void driveDist(double dist) {
        // set encoders to
        int counts = (int) Math.round(((dist / WHEEL_CIRCUMFERENCE) / GEAR_RATIO) * TICKS_PER_REV);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        setMotorPositions(counts);
        
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        drive(DRIVE_SPEED, 0, 0);
        /*
        double drivePrecision = 3.5;
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double startAngle = normAngle(angles.firstAngle);
        
        double minOffset = startAngle - OFFSET;
        double maxOffset = startAngle + OFFSET;
        
        if (minOffset <= 0) minOffset += 360;
        if (maxOffset >= 360) maxOffset -= 360;
        */
        
        while (isMotorsBusy() && opModeIsActive()) {
            /*angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            double curAngle = normAngle(angles.firstAngle);
            
            if (curAngle <= minOffset && Math.abs(curAngle - minOffset) <= drivePrecision){
                // if we go under the offset and we're within 10 of it (to prevent overflow errors)
                frontLeft.setPower(DRIVE_SPEED - 0.1);
                backLeft.setPower(DRIVE_SPEED - 0.1);
                frontRight.setPower(DRIVE_SPEED + 0.1);
                backRight.setPower(DRIVE_SPEED + 0.1);
            } else if (curAngle >= maxOffset && Math.abs(curAngle - maxOffset) <= drivePrecision) {
                // if we go over the offset and we're within 10 of it (to prevent overflow errors)
                frontRight.setPower(DRIVE_SPEED - 0.1);
                backRight.setPower(DRIVE_SPEED - 0.1);
                frontLeft.setPower(DRIVE_SPEED + 0.1);
                backLeft.setPower(DRIVE_SPEED + 0.1);
            } else {
                drive(DRIVE_SPEED, 0, 0);
            }*/
            telemetry.addData("Counts", "%d", counts);
            telemetry.addData("Front left", "%d/%d %.2f %s", frontLeft.getCurrentPosition(), frontLeft.getTargetPosition(), frontLeft.getPower(), frontLeft.isBusy() ? "Busy" : "Finished");
            telemetry.addData("Front right", "%d/%d %.2f %s", frontRight.getCurrentPosition(), frontRight.getTargetPosition(), frontRight.getPower(), frontRight.isBusy() ? "Busy" : "Finished");
            telemetry.addData("Back left", "%d/%d %.2f %s", backLeft.getCurrentPosition(), backLeft.getTargetPosition(), backLeft.getPower(), backLeft.isBusy() ? "Busy" : "Finished");
            telemetry.addData("Back right", "%d/%d %.2f %s", backRight.getCurrentPosition(), backRight.getTargetPosition(), backRight.getPower(), backRight.isBusy() ? "Busy" : "Finished");
            telemetry.update();
        }
        telemetry.update();
        drive(0, 0, 0);

        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    
    private void turn(double degrees, Direction dir) {
            // constants used for both directions
            double TURNING_SPEED = 0.3;
            double SLOWDOWN_DISTANCE = 0.75; // percentage of distance before bot slows down
            
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            double startAngle = normAngle(angles.firstAngle);
            if (dir == Direction.RIGHT) {
                double target = startAngle - degrees;
                double slowdown = startAngle - SLOWDOWN_DISTANCE * degrees;
                // Correct values if they are under 0
                target = target <= 0 ? target + 360 : target;
                slowdown = slowdown <= 0 ? slowdown + 360 : slowdown;
                turnLoop(-TURNING_SPEED, target, slowdown);
            } else if (dir == Direction.LEFT) {
                double target = startAngle + degrees;
                double slowdown = startAngle + SLOWDOWN_DISTANCE * degrees;
                // Correct values if they are over 360
                target = target >= 360 ? target - 360 : target;
                slowdown = slowdown >= 360 ? slowdown - 360 : slowdown;
                
                turnLoop(TURNING_SPEED, target, slowdown);
            }
    }
    
    private void turnLoop(double speed, double target, double slowdown) {
        double TURNING_PRECISION = 4; // amount of degrees that allows stopping
        boolean turn = true; // dummy variable because I hate do-while loops
        while (turn) {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            double curAngle = normAngle(angles.firstAngle);
            if (Math.abs(curAngle - slowdown) <= TURNING_PRECISION) speed *= 0.5;
            drive(0, 0, speed);
            if (Math.abs(curAngle - target) <= TURNING_PRECISION || !opModeIsActive()) turn = false;
        }
        drive(0, 0, 0); // stop bot
    }
    
    private boolean isMotorsBusy() {
        return backLeft.isBusy() || backRight.isBusy() || frontLeft.isBusy() || frontRight.isBusy();
    }
    
    private void setMotorPositions(int pos) {
        backLeft.setTargetPosition(pos);
        backRight.setTargetPosition(pos);
        frontLeft.setTargetPosition(pos);
        frontRight.setTargetPosition(pos);
    }
    
    private double normAngle(double angle) {
        return angle > 0 ? angle : 360 + angle;
    }
    
    private void setMotorsBrake() {
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    
    private void setMotorsFloat() {
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }
    
    // used for turning, simpler than using numbers
    enum Direction {
        LEFT,
        RIGHT;
    }
}
