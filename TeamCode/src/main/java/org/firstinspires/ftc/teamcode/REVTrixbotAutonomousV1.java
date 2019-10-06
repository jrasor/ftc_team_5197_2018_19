package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

/**
 * The Rover Ruckus 2018-19 Autonomous Opmode
 *
 *
 * By Meet 0, this would be version 1.0 of our Autonmous code.
 * Version 1.0 starts with the robot landed and knocks out a gold mineral.
 *  Robot is started with the front parallel to the lander. It faces right
 * Version 1.25 should add putting the team identifier in the depot or putting a wheel on the crater
 * Version 1.5 should have both features of 1.25
 *
 *
 * Version History
 * ======= ========
 * v 0.1  11/02/18 @Lorenzo Pedroza
 */

@Autonomous(name ="REVTrixbot: v1.0(only mineral detect)", group ="REVTrixbot")
@Disabled
public class REVTrixbotAutonomousV1 extends ModularRobotIterativeOpMode {
    private REVTrixbot robot = new REVTrixbot();

    public void init(){
        robot.dt.initHardware(hardwareMap);
        robot.goldLocator.initHardware(hardwareMap);
    }

    @Override
    public void loop() {
        //Center on And Knock off Gold
        //TODO make silver detector to count how many silver are passed to determine bearing.
        while (robot.goldLocator.getGoldPos() != Pos.MID){
            robot.dt.turnRadius(-.4, 0); //turn left
        }
        robot.dt.encoderDrive(.6, 12, 12);

        //
    }

    @Override
    public void stop() {

    }
}
