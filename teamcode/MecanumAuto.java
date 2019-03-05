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
        
        bot.turn(90, Direction.LEFT);
        bot.strafe(2, Direction.RIGHT);
        //bot.moveLift(-4);
        //driveSequence();
    }
    
    private void driveSequence() {
        bot.moveLift(6);
        // drive diagonally over 2 squares to get into depot
        bot.autoDrive(2 * SQUARE_LEN * Math.sqrt(2));
        // turn to make marker-dropper face inward
        bot.turn(45, Direction.LEFT);
        // drop marker
        bot.dropMarker();
        // strafe to the left to not hit marker
        bot.strafe(SQUARE_LEN / 3.25, Direction.LEFT);
        bot.closeMarkerServo();
        // turn all the way around toward crater
        bot.turn(175, Direction.LEFT);
        // drive to crater
        bot.autoDrive(5* SQUARE_LEN);
    }
}
