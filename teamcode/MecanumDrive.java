package org.firstinspires.ftc.TestCodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Mecanum TeleOp", group = "Test")
public class MecanumDrive extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontRight;
    private DcMotor frontLeft;
    private DcMotor backRight;
    private DcMotor backLeft;
    
    private Servo markerServo;
    private DcMotor arm;
    
    boolean markerServoOpen = false;
    String db1 = "";
    String db2 = "";

    @Override
    public void runOpMode() {
        // Assign hardware to variables from map.
        frontRight =  hardwareMap.get(DcMotor.class, "front_right");
        frontLeft  =  hardwareMap.get(DcMotor.class, "front_left");
        backRight  =  hardwareMap.get(DcMotor.class, "back_right");
        backLeft   =  hardwareMap.get(DcMotor.class, "back_left");
        markerServo = hardwareMap.get(Servo.class, "marker_servo");
        arm         = hardwareMap.get(DcMotor.class, "arm_motor");
        
        // Set direction of each motor (motors turn clockwise)
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        arm.setDirection(DcMotor.Direction.FORWARD);
        
        // set runmodes
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        
        // Wait for driver to press Play
        waitForStart();
        runtime.reset();
        
        
        while(opModeIsActive()) {
            double drive  = gamepad1.left_stick_y;
            double strafe = gamepad1.left_trigger - gamepad1.right_trigger;
            double rotate = gamepad1.right_stick_x;
            
            double frontRPower = drive - strafe + rotate;
            double frontLPower = drive + strafe - rotate;
            double backRPower  = drive + strafe + rotate;
            double backLPower  = drive - strafe - rotate;
            
            frontRight.setPower(frontRPower);
            frontLeft.setPower(frontLPower);
            backRight.setPower(backRPower);
            backLeft.setPower(backLPower);
            
            
            // Marker servo
            if (gamepad2.y && !db2.equals("y")) {
                markerServoOpen = !markerServoOpen;
                db2 = "y";
            } else if (!gamepad2.y && db2.equals("y")) {
                db2 = "";
            }
            
            if (markerServoOpen) {
                markerServo.setPosition(0);
            } else {
                markerServo.setPosition((double) 115 / 180);
            }
            
            // Arm
            double armPower = gamepad2.left_stick_y / 3;
            arm.setPower(armPower);
            
            
            telemetry.addData("Run Time", runtime.toString());
            telemetry.addData("Front Motors", "Right (%.2f) | Left (%.2f)", frontRPower, frontLPower);
            telemetry.addData("Back Motors", "Right (%.2f) | Left (%.2f)", backRPower, backLPower);
            telemetry.addData("Arm Motor", "(%.2f)", armPower);
            telemetry.update();
        }
    }
}
