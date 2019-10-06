package org.firstinspires.ftc.teamcode;

/**
 * This is NOT an opmode.
 *
 * It's an interface. This interface is a contract for all drivetrains. To be called drive-able
 * drivetrains must implement the below methods. For accessor methods requiring encoders, return -1
 * if the robot does not have encoders.
 *
 * We may need to restructure the interface hierarchy once we begin coding modular classes for strafing
 * drivetrains like mecanum.
 *
 * Version history
 * ======= =======
 * v 0.1  11/2/18 added this Java Doc. Added encoderDrive and turnAngleRadiusDrive
 */

import com.qualcomm.robotcore.hardware.Gamepad;

public interface FTCModularDrivetrainDrivable extends FTCModularizableSystems {
    //abstract public void driveLinearInches(double power, double distanceInInches);
    //abstract public void driveCurved();
    //abstract public void turn();
    void teleOpTankDrive(Gamepad driverGamepad);
    void teleOpArcadeDrive(Gamepad driverGamepad, F310JoystickInputNames.Joysticks selectedDrivingStick);
    //We would add more mandatory methods for drivetrains
    void encoderDrive(double speed, double leftInches, double rightInches);
    void turnAngleRadiusDrive(double speed, double angleRadians, double radius);
    void turnRadius(double speed, double radius);

    int getCurrentAverageDTPosition(boolean countStoppedSides); //TODO account for turning, straight lines after turning. Maybe, for specific ,straight segments, zero out Position before counting?
    //private void setModeOfAllMotors(final DcMotor.RunMode runMode); Can't be done until Java 9 is used in Android:(
}
