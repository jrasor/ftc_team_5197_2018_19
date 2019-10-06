package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * This is NOT an OpMode
 *
 * This is an interface for modular systems. A modular system should typically be independent from
 * the operation of other modular systems on the robot. Thus, to start, a modular system class should
 * be able to initialize all its own components.
 *
 * Version History
 * ======= =======
 * v 0.1 10/11/18 @Lorenzo Pedroza Added this javadoc.
 */

public interface FTCModularizableSystems {
    void initHardware(HardwareMap ahwMap);
}
