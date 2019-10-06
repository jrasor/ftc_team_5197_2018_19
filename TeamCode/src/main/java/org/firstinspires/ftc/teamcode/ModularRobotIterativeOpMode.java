package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

/**
 * This is NOT an Opmode
 *
 * This class adds the requirement of using initHardware() in an interative opmode to initialize modular
 * robot systems.
 *
 * Version history
 * ======= ======
 * v 0.1  11/02/18 @Lorenzo Pedroza Added this Javadoc
 */

public abstract class ModularRobotIterativeOpMode extends OpMode {
    //loop() is already a required Opmode method
    @Override
    abstract public void init(); //Must modify to initialize all modules/systems in robot

    @Override
    abstract public void stop();
}
