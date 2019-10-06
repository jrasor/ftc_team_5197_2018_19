package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is NOT an opmode
 *
 * This is an abstract Gerneric FTC robot class. From observation, robots do have timers to count how
 * long an opmode has been running. Not all robots use vision.
 *
 * Version history
 * ======= =======
 * v 0.1    11/02/18 @Lorenzo Pedroza. Added this javadoc
 */

public abstract class GenericFTCRobot {
    public ElapsedTime runTime;
    //potentially more common robot fields and methods

    public void init(){
        runTime = new ElapsedTime();
    }
}
