package org.firstinspires.ftc.teamcode;

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
    
	private MecanumRobot bot = new MecanumRobot(this);
	
    boolean markerServoOpen = false;
    String db1 = "";
    String db2 = "";

    @Override
    public void runOpMode() {
        // Assign hardware to variables from map.
		bot.initDrive();
		
        
        
        // Wait for driver to press Play
        waitForStart();
        runtime.reset();
        
        
        while(opModeIsActive()) {
            double drive  = gamepad1.left_stick_y;
            double strafe = gamepad1.left_trigger - gamepad1.right_trigger;
            double rotate = gamepad1.right_stick_x;
            
            bot.manualDrive(drive, strafe, rotate)
            
            
            // Marker servo
            if (gamepad2.y && !db2.equals("y")) {
                markerServoOpen = !markerServoOpen;
                db2 = "y";
            } else if (!gamepad2.y && db2.equals("y")) {
                db2 = "";
            }
            
            if (markerServoOpen) {
                bot.markerServo.setPosition(0);
            } else {
                bot.markerServo.setPosition((double) 115 / 180);
            }
            
            // Arm
            double armPower = gamepad2.left_stick_y / 3;
            bot.arm.setPower(armPower);
            
            
            telemetry.addData("Run Time", runtime.toString());
            telemetry.addData("Front Motors", "Right (%.2f) | Left (%.2f)", frontRPower, frontLPower);
            telemetry.addData("Back Motors", "Right (%.2f) | Left (%.2f)", backRPower, backLPower);
            telemetry.addData("Arm Motor", "(%.2f)", armPower);
            telemetry.update();
        }
    }
}
