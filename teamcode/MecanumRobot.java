package org.firstinspires.ftc.teamcode;

public class MecanumRobot {
	
	private ElapsedTime runtime = new ElapsedTime();
	
	private OpMode opMode;

    // Motor and servo variables
    private DcMotor frontRight;
    private DcMotor frontLeft;
    private DcMotor backLeft;
    private DcMotor backRight;
    private Servo markerServo;
	private DcMotor arm;

    
    // Variables for imu 
    private BNO055IMU imu;
    private Orientation angles;
    private Acceleration gravity;
    
    // Encoder numbers
    private final double WHEEL_CIRCUMFERENCE = 4 * Math.PI;
    private final double GEAR_RATIO = 1;
    private final int TICKS_PER_REV = 1120; 
    private final double DRIVE_SPEED = 0.75;
	
	public MecanumSetup(OpMode opMode) {
		this.opMode = opMode;
	}
	
	
	private void autoDrive(double dist) {
        // set encoders to
        int counts = (int) Math.round(((dist / WHEEL_CIRCUMFERENCE) / GEAR_RATIO) * TICKS_PER_REV);
        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
		setMotorPositions(counts);
        
        setMode(DcMotor.RunMode.RUN_TO_POSITION);

        setPower(DRIVE_SPEED);
        
        while (isMotorsBusy() && opMode.opModeIsActive()) {
            encoderTel(counts);
        }
        telemetry.update();
        setPower(0);

        setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
	
	public void manualDrive(double drive, double strafe, double rotate) {
		double frontRPower = drive - strafe + rotate;
        double frontLPower = drive + strafe - rotate;
        double backRPower  = drive + strafe + rotate;
        double backLPower  = drive - strafe - rotate;
            
        frontRight.setPower(frontRPower);
        frontLeft.setPower(frontLPower);
        backRight.setPower(backRPower);
        backLeft.setPower(backLPower);
	}
	
	private void turn(double degrees, Direction dir) {
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
                turnLoop(TURNING_SPEED, target, slowdown);
            } else if (dir == Direction.LEFT) {
                double target = startAngle + degrees;
                double slowdown = startAngle + SLOWDOWN_DISTANCE * degrees;
                // Correct values by subtracting 360 if they are over 360
                target = target >= 360 ? target - 360 : target;
                slowdown = slowdown >= 360 ? slowdown - 360 : slowdown;
                
                turnLoop(-TURNING_SPEED, target, slowdown);
            }
    }
    
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
            //drive(0, 0, speed);
            if (Math.abs(curAngle - target) <= TURNING_PRECISION || !opMode.opModeIsActive()) turn = false;
        }
        setPower(0); // stop bot
    }
	
	private void strafe(double dist, Direction direction) {
    // set encoders to
        int counts = (int) Math.round(((dist / WHEEL_CIRCUMFERENCE) / GEAR_RATIO) * TICKS_PER_REV * 1.5);
		setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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
            encoderTel(counts);
        }
        telemetry.update();
        drive(0, 0, 0);

        setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
	
	public void initDrive() {
		
		// initialize hardware
        frontRight  = opMode.hardwareMap.get(DcMotor.class, "front_right");
        frontLeft   = opMode.hardwareMap.get(DcMotor.class, "front_left");
        backRight   = opMode.hardwareMap.get(DcMotor.class, "back_right");
        backLeft    = opMode.hardwareMap.get(DcMotor.class, "back_left");
        markerServo = opMode.hardwareMap.get(Servo.class, "marker_servo");
        arm         = opMode.hardwareMap.get(DcMotor.class, "arm_motor");


        
        // set motor directions
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        arm.setDirection(DcMotor.Direction.FORWARD);

        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        
        // set motors runmodes
        setMode(DcMotor.RunMode.RUN_USING_ENCODER;
	}
	
	public void initVuforia() {
		VuforiaLocalizer vuforia;
		VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

		// vuforia setup
        //parameters.vuforiaLicenseKey = "license key here";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK; // use back camera
        //vuforia = ClassFactory.createVuforiaLocalizer(parameters); // create the vuforia localizer
        //VuforiaTrackables roverTrackables = vuforia.loadTrackablesFromAsset(""); // TODO: find the name for the trackables
	}
	
	public void initImu() {
		// imu setup
        BNO055IMU.Parameters imuParameters = new BNO055IMU.Parameters();
        imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES; // wouldn't return radians even when I set this to radians?
        imuParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imuParameters.mode = BNO055IMU.SensorMode.IMU;
        imuParameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // i honestly have no idea if this file was created correctly since I'm using OnBotJava
        imuParameters.loggingEnabled = true;
        imuParameters.loggingTag = "IMU";
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(imuParameters);
	}
	
	
	public void setMode(DcMotor.RunMode mode) {
		frontRight.setMode(mode);
        frontLeft.setMode(mode);
        backRight.setMode(mode);
        backLeft.setMode(mode);
	}
	
	public void setPower(double power) {
		frontRight.setPower(power);
        frontLeft.setPower(power);
        backRight.setPower(power);
        backLeft.setPower(power);
	}
	
	    // check if any motor is currently busy
    private boolean isMotorsBusy() {
        return backLeft.isBusy() || backRight.isBusy() || frontLeft.isBusy() || frontRight.isBusy();
    }
    
    // simple function to allow setting all motors to the same position
    private void setMotorPositions(int pos) {
        backLeft.setTargetPosition(pos);
        backRight.setTargetPosition(pos);
        frontLeft.setTargetPosition(pos);
        frontRight.setTargetPosition(pos);
    }
    
    // adds 360 to an angle if it's under 0, done because the gyro returns negative angles between 180 and 360
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

	private void encoderTel(int counts) {
        telemetry.addData("Counts", "%d", counts);
        telemetry.addData("Front left", "%d/%d %.2f %s", frontLeft.getCurrentPosition(), frontLeft.getTargetPosition(), frontLeft.getPower(), frontLeft.isBusy() ? "Busy" : "Finished");
        telemetry.addData("Front right", "%d/%d %.2f %s", frontRight.getCurrentPosition(), frontRight.getTargetPosition(), frontRight.getPower(), frontRight.isBusy() ? "Busy" : "Finished");
        telemetry.addData("Back left", "%d/%d %.2f %s", backLeft.getCurrentPosition(), backLeft.getTargetPosition(), backLeft.getPower(), backLeft.isBusy() ? "Busy" : "Finished");
        telemetry.addData("Back right", "%d/%d %.2f %s", backRight.getCurrentPosition(), backRight.getTargetPosition(), backRight.getPower(), backRight.isBusy() ? "Busy" : "Finished");
        telemetry.update();
    }
}