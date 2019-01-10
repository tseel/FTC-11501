package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;


@Autonomous(name = "Mecanum Autonomous", group = "Live")
public class MecanumAuto extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    
    final double markerClose = 0;
    final double markerOpen = (double) 115 / 180;
	
	private MecanumRobot bot = new MecanumRobot(this);

    @Override
    public void runOpMode() {
        /*
         * SETUP
         */
        
		bot.initDrive();
		bot.initVuforia();
		bot.initImu();
        
        // wait for user to start opmode
        waitForStart();
        runtime.reset();
        
		// distances for this probably need to be worked out
        
        // drive diagonally over 2 squares to get into depot
        bot.autoDrive(2 * 23.5 * Math.sqrt(2));
        // drop marker
        markerServo.setPosition(markerOpen);
        // strafe to the left to not hit marker
        bot.strafe(23.5, Direction.LEFT);
        // turn toward crater
        turn(45, Direction.RIGHT);
        markerServo.setPosition(markerClose);
        // drive to crater
        driveDist(4.5 * 23.5);
    }
    
    
}
