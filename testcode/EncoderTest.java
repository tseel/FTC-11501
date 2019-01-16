package org.firstinspires.ftc.testcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "Encoder Test", group = "Test")
public class EncoderTest extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    // Motor and servo variables
    private DcMotor frontRight;
    private DcMotor frontLeft;
    private DcMotor backLeft;
    private DcMotor backRight;
    private Servo markerServo;
    
    // Encoder numbers
    private final double WHEEL_CIRCUMFERENCE = 4 * Math.PI;
    private final double GEAR_RATIO = 40;
    private final int TICKS_PER_REV = 1120; 

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
        
        // wait for user to start opmode
        waitForStart();
        runtime.reset();
        
        while (opModeIsActive()) {
            telemetry.addData("Front left", "%d", frontLeft.getCurrentPosition());
            telemetry.addData("Front right", "%d", frontRight.getCurrentPosition());
            telemetry.addData("Back left", "%d", backLeft.getCurrentPosition());
            telemetry.addData("Back right", "%d", backRight.getCurrentPosition());
            telemetry.update();
        }
    }
}
