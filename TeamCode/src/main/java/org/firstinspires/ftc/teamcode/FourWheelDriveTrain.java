package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * This is NOT and opmode
 *
 * This is the code for a modular robot part, a drivetrain. A drive train object can be
 * added to a robot class if needed, just like a sensor or software service object.
 * The code below is for a four wheel drive train. It is almost identical to a two drive train
 * class, the only difference being it uses four motors.
 *
 * Version history
 * ======  =======
 * v 0.1    11/02/18 @Lorenzo Pedroza. Implemented methods for endoderDrive and turnAngleRadiusDrive, and accessor methods for encoder counts. Also added method for turning a radius //TODO Test them
 * v 0.3    11/03/18 @Lorenzo Pedroza  Fixed encoderDrive
 */

public class FourWheelDriveTrain extends ModularDriveTrain{
    // REVTrix specific motor and actuator members.
    private DcMotor FrontLeftDrive   = null;
    private DcMotor FrontRightDrive  = null;
    private DcMotor RearLeftDrive    = null;
    private DcMotor RearRightDrive   = null;

    private String frontLeftMotorName = null;
    private String frontRightMotorName = null;
    private String rearLeftMotorName = null;
    private String rearRightMotorName = null;

    FourWheelDriveTrain(double COUNTS_PER_MOTOR_REV, double DRIVE_GEAR_REDUCTION,
                        double WHEEL_DIAMETER_INCHES, double DRIVE_WHEEL_SEPARATION,
                        DcMotor.RunMode RUNMODE, String frontLeftMotorName, String frontRightMotorName,
                        String rearLeftMotorName, String rearRightMotorName){
        super(COUNTS_PER_MOTOR_REV, DRIVE_GEAR_REDUCTION, WHEEL_DIAMETER_INCHES, DRIVE_WHEEL_SEPARATION, RUNMODE);
        this.frontLeftMotorName = frontLeftMotorName;
        this.frontRightMotorName = frontRightMotorName;
        this.rearLeftMotorName = rearLeftMotorName;
        this.rearRightMotorName = rearRightMotorName;
    }

    private void setModeOfAllMotors(final DcMotor.RunMode runMode) {
        FrontLeftDrive.setMode(runMode);
        FrontRightDrive.setMode(runMode);
        RearLeftDrive.setMode(runMode);
        RearRightDrive.setMode(runMode);

    }

    public void initHardware(HardwareMap ahwMap){

        FrontLeftDrive = ahwMap.get(DcMotor.class, frontLeftMotorName);
        FrontRightDrive = ahwMap.get(DcMotor.class, frontRightMotorName);
        RearLeftDrive = ahwMap.get(DcMotor.class, rearLeftMotorName);
        RearRightDrive = ahwMap.get(DcMotor.class, rearRightMotorName);

        FrontLeftDrive.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if
        // using AndyMark motors
        FrontRightDrive.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if
        // using AndyMark motors
        RearLeftDrive.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if
        // using AndyMark motors
        RearRightDrive.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if
        // using AndyMark motors

        // Set all motors to zero power
        FrontLeftDrive.setPower(0);
        FrontRightDrive.setPower(0);
        RearLeftDrive.setPower(0);
        RearRightDrive.setPower(0);

        if(RUNMODE == DcMotor.RunMode.RUN_USING_ENCODER){  //stops motors until mode changed; so do before setting runmode to RUN_USING_Encoder
            setModeOfAllMotors(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        // Set both motors to run with encoders.
        setModeOfAllMotors(RUNMODE);

    }

    public void teleOpTankDrive(Gamepad driverGamepad){
        // Run wheels in tank mode (note: The joystick goes negative when pushed forwards,
        // so negate it) Negate it at function call
        leftPower = -driverGamepad.left_stick_y;
        rightPower = -driverGamepad.right_stick_y;
        FrontLeftDrive.setPower(leftPower);
        FrontRightDrive.setPower(rightPower);
        RearLeftDrive.setPower(leftPower);
        RearRightDrive.setPower(rightPower);
    }

    public void teleOpArcadeDrive(Gamepad driverGamepad, F310JoystickInputNames.Joysticks selectedDrivingStick){

        //float leftPower = 0f; //may want to moveRotations these as field variables. Probably should not initialize every time.
        //float rightPower = 0f;

        if(selectedDrivingStick == F310JoystickInputNames.Joysticks.LEFT_STICK){
            leftPower = -driverGamepad.left_stick_y - -driverGamepad.left_stick_x;
            rightPower = -driverGamepad.left_stick_y + -driverGamepad.left_stick_x;
        }

        else if(selectedDrivingStick == F310JoystickInputNames.Joysticks.RIGHT_STICK) {
            leftPower = -driverGamepad.right_stick_y - -driverGamepad.right_stick_x;
            rightPower = -driverGamepad.right_stick_y + -driverGamepad.right_stick_x;
        }

        FrontLeftDrive.setPower(leftPower);
        FrontRightDrive.setPower(rightPower);
        RearLeftDrive.setPower(leftPower);
        RearRightDrive.setPower(rightPower);
    }
    public void encoderDrive(double speed,
                             double leftInches, double rightInches) {
        if (RUNMODE != DcMotor.RunMode.RUN_USING_ENCODER){
            return; //Cannot use method if no encoders.
        }

        int newLeftTarget = 0;
        int newRightTarget = 0;

        setModeOfAllMotors(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //do to correct differences in rotations

        newLeftTarget = getCurrentAverageLeftDTPosition() + (int)
                (leftInches * COUNTS_PER_INCH);

        newRightTarget = getCurrentAverageRightDTPosition() + (int)(rightInches * COUNTS_PER_INCH);

        FrontLeftDrive.setTargetPosition(newLeftTarget);
        RearLeftDrive.setTargetPosition(newLeftTarget);

        FrontRightDrive.setTargetPosition(newRightTarget);
        RearRightDrive.setTargetPosition(newRightTarget);

        setModeOfAllMotors(DcMotor.RunMode.RUN_TO_POSITION);


        FrontLeftDrive.setPower(Math.abs(speed));
        RearLeftDrive.setPower(Math.abs(speed));
        FrontRightDrive.setPower(Math.abs(speed));
        RearRightDrive.setPower(Math.abs(speed));

        //Wait for motors to moveRotations to position
        while((FrontLeftDrive.isBusy() && RearLeftDrive.isBusy() && (FrontRightDrive.isBusy() && RearRightDrive.isBusy()))){}

        //Set motor speed to zero
        FrontLeftDrive.setPower(0);
        RearLeftDrive.setPower(0);
        FrontRightDrive.setPower(0);
        RearRightDrive.setPower(0);

        // Turn off RUN_TO_POSITION
        setModeOfAllMotors(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void powerDrive(double leftPower, double rightPower){
        FrontLeftDrive.setPower(leftPower);
        RearLeftDrive.setPower(leftPower);
        FrontRightDrive.setPower(rightPower);
        RearRightDrive.setPower(rightPower);
    }

    public void driveStraight(double speed, double inches){
        encoderDrive(speed, inches, inches);
    } //Maybe don't need this

    public void turnAngleRadiusDrive (double speed, double angleRadians, double radius){

        if (RUNMODE != DcMotor.RunMode.RUN_USING_ENCODER){
            return; //Cannot use method if no encoders.
        }

        double leftArc = (radius - DRIVE_WHEEL_SEPARATION/2.0)*angleRadians;
        double rightArc = (radius + DRIVE_WHEEL_SEPARATION/2.0)*angleRadians;

        double leftSpeed = speed * (radius - DRIVE_WHEEL_SEPARATION/2.0)/
                (radius + DRIVE_WHEEL_SEPARATION);
        double rightSpeed = speed;

        int newLeftTarget;
        int newRightTartget;

        setModeOfAllMotors(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        newLeftTarget = (int) (leftArc * COUNTS_PER_INCH);
        newRightTartget = (int) (rightArc * COUNTS_PER_INCH);

        FrontLeftDrive.setTargetPosition(newLeftTarget);
        RearLeftDrive.setTargetPosition(newLeftTarget);
        FrontRightDrive.setTargetPosition(newRightTartget);
        FrontLeftDrive.setTargetPosition(newRightTartget);

        setModeOfAllMotors(DcMotor.RunMode.RUN_TO_POSITION);

        FrontLeftDrive.setPower(Math.abs(leftSpeed));
        RearLeftDrive.setPower(Math.abs(leftSpeed));
        FrontRightDrive.setPower(Math.abs(rightSpeed));
        RearRightDrive.setPower(Math.abs(rightSpeed));

        //Wait for motors to moveRotations to position
        while((FrontLeftDrive.isBusy() && RearLeftDrive.isBusy() && (FrontRightDrive.isBusy() && RearRightDrive.isBusy())));

        //Stop All motors

        FrontLeftDrive.setPower(0);
        FrontRightDrive.setPower(0);
        RearLeftDrive.setPower(0);
        RearRightDrive.setPower(0);

        setModeOfAllMotors(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void turnRadius(double speed, double radius){
        //Calculate how much faster one side must rotate than other
        double leftSpeed = speed * (radius - DRIVE_WHEEL_SEPARATION/2.0)/
                (radius + DRIVE_WHEEL_SEPARATION);
        double rightSpeed = speed;

        FrontLeftDrive.setPower(Math.abs(leftSpeed));
        RearLeftDrive.setPower(Math.abs(leftSpeed));
        FrontRightDrive.setPower(Math.abs(rightSpeed));
        RearRightDrive.setPower(Math.abs(rightSpeed));
    }


    //Methods to access drivetrain motor values

    public int getCurrentAverageDTPosition(boolean countStoppedDTSides) { //TODO Work on getting encoder readins //
        boolean leftDriveMoving = FrontLeftDrive.getPowerFloat() && RearLeftDrive.getPowerFloat(); //TODO see if this is the correct method to determine if a motor is moving
        boolean rightDriveMoving = FrontRightDrive.getPowerFloat() && RearRightDrive.getPowerFloat();
        if(RUNMODE == DcMotor.RunMode.RUN_USING_ENCODER){
            if (!countStoppedDTSides && !(leftDriveMoving && rightDriveMoving)) {//don't return these values if left and right motors already moving
                if (leftDriveMoving){
                    return getCurrentAverageLeftDTPosition();
                }
                else if(rightDriveMoving){
                    return getCurrentAverageRightDTPosition();
                }
            }
            return (FrontLeftDrive.getCurrentPosition() + FrontRightDrive.getCurrentPosition() +
                    RearLeftDrive.getCurrentPosition() + RearRightDrive.getCurrentPosition()) / 4;


        }
        return -1; //cannot use method when they are no encoders used.
    }

    public int getCurrentAverageLeftDTPosition() { //TODO Work on getting encoder readins //
        if(RUNMODE == DcMotor.RunMode.RUN_USING_ENCODER){
            return (FrontLeftDrive.getCurrentPosition() + RearLeftDrive.getCurrentPosition())/2;
        }
        return -1; //cannot use method when they are no encoders used.
    }

    public int getCurrentAverageRightDTPosition() { //TODO Work on getting encoder readins //
        if(RUNMODE == DcMotor.RunMode.RUN_USING_ENCODER){
            return (FrontRightDrive.getCurrentPosition() + RearRightDrive.getCurrentPosition())/2;
        }
        return -1; //cannot use method when they are no encoders used.
    }

    public int getCurrentFrontLeftDrivePosition(){
        if(RUNMODE == DcMotor.RunMode.RUN_USING_ENCODER){
            return FrontLeftDrive.getCurrentPosition();
        }
        return -1; //cannot use method when they are no encoders used.
    }

    public int getCurrentFrontRightDrivePosition(){
        if(RUNMODE == DcMotor.RunMode.RUN_USING_ENCODER){
            return FrontRightDrive.getCurrentPosition();
        }
        return -1; //cannot use method when they are no encoders used.
    }

    public int getCurrentRearLeftDrivePosition(){
        if(RUNMODE == DcMotor.RunMode.RUN_USING_ENCODER){
            return RearLeftDrive.getCurrentPosition();
        }
        return -1; //cannot use method when they are no encoders used.
    }

    public int getCurrentRearRightDrivePosition(){
        if(RUNMODE == DcMotor.RunMode.RUN_USING_ENCODER){
            return RearRightDrive.getCurrentPosition();
        }
        return -1; //cannot use method when they are no encoders used.
    }
}

