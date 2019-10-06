package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

/**
 * An opmode to callibrate a REVTrix bot
 *
 * Version history
 * ======= ========
 * v 0.1    11/02/18 @Lorenzo Pedroza. Added this Javadoc
 * v 1.0    11/03/18 @Lorenzo Pedroza. Fixed callibrare so it only moves once.
 */

@Autonomous(name="Calibrate", group="REVTrixbot")
@Disabled
public class REVTrix_Callibrate_Iterative extends OpMode {

    REVTrixbot robot = new REVTrixbot();

    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.2;

    @Override
    public void init() {
        robot.dt.initHardware(hardwareMap);
    }


    @Override
    public void start() {
        robot.dt.encoderDrive(DRIVE_SPEED, -3, -3);
    }

    @Override
    public void loop() {

    }
}
