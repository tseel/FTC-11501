package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Mecanum Autonomous Far", group = "Live")
public class MecanumAutoFar extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    
    private final double SQUARE_LEN = 23.5;
    
    private MecanumRobot bot = new MecanumRobot(this);

    @Override
    public void runOpMode() {
        /*
         * SETUP
         */
        
        bot.initDrive();
        // bot.initVuforia();
        bot.initImu();
        
        // wait for user to start opmode
        waitForStart();
        runtime.reset();
        
        driveSequence();
    }
    
    private void driveSequence() {
        // start hanging
        // go down
        bot.moveLift(9.7);
        // unhook from lander
        bot.strafe(6, Direction.LEFT);
        bot.autoDrive(3);
        bot.strafe(6, Direction.RIGHT);
        
        // correct for minor disorientation
        bot.turn(5, Direction.LEFT);
        
        // turn toward wall
        bot.turn(45, Direction.LEFT);
        
        // drive toward wall
        bot.autoDrive(2 * SQUARE_LEN);
        
        // turn toward depot
        bot.turn(81, Direction.LEFT);
        
        // drive to depot
        bot.autoDrive(3 * SQUARE_LEN);
        bot.turn(45, Direction.RIGHT);
        bot.dropMarker();
        /*
        bot.turn(45, Direction.LEFT);
        bot.closeMarkerServo();
        bot.autoDrive(-SQUARE_LEN);*/
    }
}
