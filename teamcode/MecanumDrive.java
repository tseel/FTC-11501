package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name = "Mecanum TeleOp", group = "Test")
public class MecanumDrive extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    
    // hardware initialization
    private MecanumRobot bot = new MecanumRobot(this);
    
    // for the marker servo's button toggle
    boolean markerServoOpen = false;
    
    // debounce variables, one for each controller, allows for button toggles
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
            // driving
            double drive  = gamepad1.left_stick_y;
            double strafe = gamepad1.right_trigger - gamepad1.left_trigger;
            double rotate = gamepad1.right_stick_x;
            
            bot.manualDrive(drive, strafe, rotate);
            
            // Marker servo
            if (gamepad2.y && !db2.equals("y")) {
                markerServoOpen = !markerServoOpen;
                db2 = "y"; // set debounce to y
            } else if (!gamepad2.y && db2.equals("y")) {
                db2 = ""; // clear debounce
            }
            
            if (markerServoOpen) {
                bot.markerServo.setPosition(0);
            } else {
                bot.markerServo.setPosition((double) 115 / 180);
            }
            
            // Arm
            double armPower = gamepad2.left_stick_y / 3;
            bot.arm.setPower(armPower);
        }
    }
}
