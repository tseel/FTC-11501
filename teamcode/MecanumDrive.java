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

    boolean backward = false;

    @Override
    public void runOpMode() {
        // Assign hardware to variables from map.
        bot.initDrive();
        
        // Wait for driver to press Play
        waitForStart();
        runtime.reset();
        
        while(opModeIsActive()) {
            // driving
            double drive  = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_trigger - gamepad1.right_trigger;
            double rotate = -gamepad1.right_stick_x;
            
            if (!backward) {
                bot.manualDrive(drive, strafe, rotate);
            } else {
                bot.manualDrive(-drive, -strafe, -rotate);
            }
            
            if (gamepad1.y && !db1.equals("y")) {
                backward = !backward;
                db1 = "y";
            } else if (!gamepad1.y && db1.equals("y")) {
                db1 = "";
            }
            
            // Marker servo
            if (gamepad2.y && !db2.equals("y")) {
                markerServoOpen = !markerServoOpen;
                db2 = "y"; // set debounce to y
            } else if (!gamepad2.y && db2.equals("y")) {
                db2 = ""; // clear debounce
            }
            
            if (markerServoOpen) {
                bot.markerServo.setPosition(bot.markerOpen);
            } else {
                bot.markerServo.setPosition(bot.markerClose);
            }
            
            if (!bot.isLiftBusy() && gamepad2.a) {
                bot.moveLift(-4.5);
            }
            
            double liftUpPower = gamepad2.left_stick_y / 2;
            double liftDownPower = gamepad2.right_stick_y / 2;
            bot.liftUp.setPower(-liftUpPower * 1.125);
            bot.liftDown.setPower(liftDownPower);
        }
    }
}
