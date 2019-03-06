package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "DropBot", group = "Live")
public class DropBot extends LinearOpMode {

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
        // bot.initImu();
        
        // wait for user to start opmode
        waitForStart();
        runtime.reset();
        
        bot.moveLift(9.9);
        bot.strafe(6 , Direction.LEFT);
        bot.autoDrive(3);
        bot.strafe(6, Direction.RIGHT);
    }
}