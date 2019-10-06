package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * This is NOT an opmode.
 *
 * This a parent abstract class for drivetrains. It contains fields used by all drivetraains
 *
 * Version history
 * ======= ========
 * v 0.1 11/02/18 @Lorenzo Pedroza. Added this Javadoc
 */

public abstract class ModularDriveTrain extends Thread implements FTCModularDrivetrainDrivable {
    // drivetrain specific measurements
    protected final double     COUNTS_PER_MOTOR_REV;
    protected final double     DRIVE_GEAR_REDUCTION;

    // drivetrain specific drive train members.
    protected final double     WHEEL_DIAMETER_INCHES;
    protected final double     DRIVE_WHEEL_SEPARATION;
    protected final DcMotor.RunMode RUNMODE; //made protected so child classes (actual drivetrains, can access these constants)
    protected final double     COUNTS_PER_INCH;

    protected float leftPower = 0f; //TODO have methods for encoders?
    protected float rightPower = 0f;


    //abstract public void initHardware(HardwareMap ahwMap);
    //abstract public void driveLinearInches(double power, double distanceInInches);
    //abstract public void driveCurved();
    //abstract public void turn();
    //We would add more mandatory functions for drivetrains

    ModularDriveTrain( double COUNTS_PER_MOTOR_REV, double DRIVE_GEAR_REDUCTION, double WHEEL_DIAMETER_INCHES, double DRIVE_WHEEL_SEPARATION, DcMotor.RunMode RUNMODE){
        this.COUNTS_PER_MOTOR_REV = COUNTS_PER_MOTOR_REV;
        this.DRIVE_GEAR_REDUCTION = DRIVE_GEAR_REDUCTION;
        this.WHEEL_DIAMETER_INCHES = WHEEL_DIAMETER_INCHES;
        this.DRIVE_WHEEL_SEPARATION = DRIVE_WHEEL_SEPARATION;
        this.RUNMODE = RUNMODE;
        COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                (WHEEL_DIAMETER_INCHES * 3.1415);
    }


}
