package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Mecanum Autonomous", group = "Live")
public class MecanumAuto extends LinearOpMode {

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
        // distances for this probably need to be worked out
        
        // drive diagonally over 2 squares to get into depot
        bot.autoDrive(2 * SQUARE_LEN * Math.sqrt(2));
        bot.turn(45, Direction.LEFT);
        // drop marker
        bot.dropMarker();
        // strafe to the left to not hit marker
        bot.strafe(SQUARE_LEN / 3.25, Direction.LEFT);
        bot.closeMarkerServo();
        // turn all the way around toward crater
        bot.turn(175, Direction.LEFT);
        // drive to crater
        bot.autoDrive(4.5 * SQUARE_LEN);
    }
}
