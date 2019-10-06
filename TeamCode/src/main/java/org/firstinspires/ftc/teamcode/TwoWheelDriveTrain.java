package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

/**
 * This is NOT and opmode
 *
 * This is the code for a modular robot part, a drivetrain. A drive train object can be
 * added to a robot class if needed, just like a sensor or software service object.
 * The code below is for a two wheel drive train.
 *
 * Version history
 * ======  =======
 * v 0.1    11/02/18 @Lorenzo Pedroza. Implemented methods for endoderDrive and turnAngleRadiusDrive, and accessor methods for encoder counts //TODO Test them
 * v 0.5    11/03/18 @Lorenzo Pedroza. Added while statements in encoder methods to allow motors to moveRotations to position
 * v 0.6    11/07/18 Ir is now easier to give motor unique names; names are now required parameter
 */


public class TwoWheelDriveTrain extends ModularDriveTrain {
    private DcMotor leftDrive   = null;
    private DcMotor rightDrive  = null;
    private String leftMotorName = null;
    private String rightMotorName = null;

    TwoWheelDriveTrain(double COUNTS_PER_MOTOR_REV, double DRIVE_GEAR_REDUCTION,
                       double WHEEL_DIAMETER_INCHES, double DRIVE_WHEEL_SEPARATION,
                       DcMotor.RunMode RUNMODE, String leftMotorName, String rightMotorName){
        super(COUNTS_PER_MOTOR_REV, DRIVE_GEAR_REDUCTION, WHEEL_DIAMETER_INCHES, DRIVE_WHEEL_SEPARATION, RUNMODE);
        this.leftMotorName = leftMotorName;
        this.rightMotorName = rightMotorName;
    }

    private void setModeOfAllMotors(final DcMotor.RunMode runMode) {
        leftDrive.setMode(runMode);
        rightDrive.setMode(runMode);
    }

    public void initHardware(HardwareMap ahwMap){
        leftDrive = ahwMap.get(DcMotor.class, leftMotorName);
        rightDrive = ahwMap.get(DcMotor.class, rightMotorName);

        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);

        leftDrive.setPower(0);
        rightDrive.setPower(0);

        if(RUNMODE == DcMotor.RunMode.RUN_USING_ENCODER){  //stops motors until mode changed; so do before setting runmode to RUN_USING_Encoder
            setModeOfAllMotors(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        setModeOfAllMotors(RUNMODE);
    }

    public void teleOpTankDrive(Gamepad driverGamepad) {
        // Run wheels in tank mode (note: The joystick goes negative when pushed forwards,
        // so negate it) Negate it at function call
        leftPower = -driverGamepad.left_stick_y; //may seem unnecessary to assign leftPower, but this is for consistency.
        rightPower = -driverGamepad.right_stick_y;

        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);
    }

    public void teleOpArcadeDrive(Gamepad driverGamepad, F310JoystickInputNames.Joysticks selectedDrivingStick) {

        if(selectedDrivingStick == F310JoystickInputNames.Joysticks.LEFT_STICK){ //in future, consider moving this to parent class
            leftPower = -driverGamepad.left_stick_y - -driverGamepad.left_stick_x;
            rightPower = -driverGamepad.left_stick_y + -driverGamepad.left_stick_x;
        }

        else if(selectedDrivingStick == F310JoystickInputNames.Joysticks.RIGHT_STICK) {
            leftPower = -driverGamepad.right_stick_y - -driverGamepad.right_stick_x;
            rightPower = -driverGamepad.right_stick_y + -driverGamepad.right_stick_x;
        }

        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);

    }

    public void encoderDrive(double speed,
                             double leftInches, double rightInches) {
        if (RUNMODE != DcMotor.RunMode.RUN_USING_ENCODER){
            return; //Cannot use method if no encoders.
        }

        int newLeftTarget = 0;
        int newRightTarget = 0;

        newLeftTarget = getCurrentLeftDrivePosition() + (int)
                (leftInches * COUNTS_PER_INCH);

        newRightTarget = getCurrentRightDrivePosition() + (int)(rightInches * COUNTS_PER_INCH);

        leftDrive.setTargetPosition(newLeftTarget);
        rightDrive.setTargetPosition(newRightTarget);

        setModeOfAllMotors(DcMotor.RunMode.RUN_TO_POSITION);


        leftDrive.setPower(Math.abs(speed));
        rightDrive.setPower(Math.abs(speed));

        //Wait for motors to moveRotations to position
        while(leftDrive.isBusy() && rightDrive.isBusy()){}

        //Set motor speed to zero
        leftDrive.setPower(0);
        rightDrive.setPower(0);

        // Turn off RUN_TO_POSITION
        setModeOfAllMotors(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void powerDrive(double leftPower, double rightPower){
        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);
    }

    public void turnAngleRadiusDrive (double speed, double angleRadians, double radius){
        if (RUNMODE != DcMotor.RunMode.RUN_USING_ENCODER){
            return; //Cannot use method if no encoders.
        }

        double leftArc = (radius - DRIVE_WHEEL_SEPARATION/2.0)*angleRadians;
        double rightArc = (radius + DRIVE_WHEEL_SEPARATION/2.0)*angleRadians;

        double leftSpeed = speed * (radius - DRIVE_WHEEL_SEPARATION/2.0)/
                (radius + DRIVE_WHEEL_SEPARATION);
        double rightSpeed = speed;
        //TODO consider writing to global leftPower and right power

        int newLeftTarget;
        int newRightTarget;

        setModeOfAllMotors(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        newLeftTarget = (int) (leftArc * COUNTS_PER_INCH);
        newRightTarget = (int) (rightArc * COUNTS_PER_INCH);

        leftDrive.setTargetPosition(newLeftTarget);
        rightDrive.setTargetPosition(newRightTarget);

        setModeOfAllMotors(DcMotor.RunMode.RUN_TO_POSITION);

        leftDrive.setPower(Math.abs(leftSpeed));
        rightDrive.setPower(Math.abs(rightSpeed));

        //Wait for motors to moveRotations to position
        while(leftDrive.isBusy() && rightDrive.isBusy()){}

        //Stop all motors
        leftDrive.setPower(0);
        rightDrive.setPower(0);

        setModeOfAllMotors(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void turnRadius(double speed, double radius){
        double leftSpeed = speed * (radius - DRIVE_WHEEL_SEPARATION/2.0)/
                (radius + DRIVE_WHEEL_SEPARATION);
        double rightSpeed = speed;

        leftDrive.setPower(leftSpeed);
        rightDrive.setPower(rightSpeed);
    }

    public int getCurrentAverageDTPosition(boolean countStoppedDTSides) { //TODO Work on getting encoder readins //
        boolean leftDriveMoving = leftDrive.getPowerFloat(); //TODO see if this is the correct method to determine if a motor is moving
        boolean rightDriveMoving = rightDrive.getPowerFloat();
        if(RUNMODE == DcMotor.RunMode.RUN_USING_ENCODER){
            if (!countStoppedDTSides && !(leftDriveMoving && rightDriveMoving)) {//don't return these values if left and right motors already moving
                if (leftDriveMoving){
                    return getCurrentLeftDrivePosition();
                }
                else if(rightDriveMoving){
                    return getCurrentRightDrivePosition();
                }
            }
            return (leftDrive.getCurrentPosition() + rightDrive.getCurrentPosition())/2;
        }
        return -1; //cannot use method when they are no encoders used.
    }

    public int getCurrentLeftDrivePosition(){
        if(RUNMODE == DcMotor.RunMode.RUN_USING_ENCODER){
            return leftDrive.getCurrentPosition();
        }
        return -1; //cannot use method when they are no encoders used.
    }

    public int getCurrentRightDrivePosition(){
        if(RUNMODE == DcMotor.RunMode.RUN_USING_ENCODER){
            return rightDrive.getCurrentPosition();
        }
        return -1; //cannot use method when they are no encoders used.
    }



}
