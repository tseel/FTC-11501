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
        // start hanging
        // go down
        bot.moveLift(9.7);
        // unhook from lander
        bot.strafe(6, Direction.LEFT);
        bot.autoDrive(3);
        bot.strafe(6, Direction.RIGHT);
        
        // correct for minor disorientation
        bot.turn(5, Direction.LEFT);

        // drive diagonally over 2 squares to get into depot
        bot.autoDrive((2 * SQUARE_LEN * Math.sqrt(2)) - 4);
        // turn to make marker-dropper face inward
        bot.turn(45, Direction.RIGHT);
        // drop marker
        bot.dropMarker();
        // strafe to the right to not hit marker
        bot.strafe(SQUARE_LEN / 3.75, Direction.RIGHT);
        bot.closeMarkerServo();
        
        // drive out of square to get out of way
        bot.autoDrive(-SQUARE_LEN);

        /* drive to crater
        bot.turn(75, Direction.RIGHT);
        bot.autoDrive(5* SQUARE_LEN);
        */
    }
}
