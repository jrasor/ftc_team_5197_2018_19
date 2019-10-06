package org.firstinspires.ftc.teamcode;

import android.support.annotation.Nullable;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * THIS IS NOT AN OPMODE
 * IT is a way of creating a Dc motorized actuator that has limits to movement.
 * *IMPORTANT*
 * If ony using encoders and/or an upper limit switch(no lower limit switch), it is required
 * that before initizalization, the actuator must be manually moved to a zero position for the robot to
 * capture upon initialization. Otherwise, the acutator will not function properly which may lead to
 * damage to the robot. This is not necesaary if you have a lower limit switch; nontheless, if the acutator
 * is going to actively pulled or pushed away from the zero position during inttializaiotn (like due to
 * hanging the robot), initizlaiziton
 * should be done before placing the actuator in this condition on the field to be safe.
 *
 * Version History
 * =================
 * v0.1  @Author Lorenzo Pedroza 11/24/18 //
 * v0.2  @Authot Lorenzo Pedroza 11/26/18 More additions
 * v0.4  @Author Lorenzo Pedroza 11/28/18 Works for only an encoder //TODO test limiy swtiches and make method to calculate time for watch dog also/or try making this class a thread.
 * v0.5  @Author Lorenzo Pedroza 11/28/18 Limit Switch code development
 * */


public class LimitedDcMotorDrivenActuator extends Thread implements FTCModularizableSystems{
    protected DcMotor motor;
    private final double INIT_MOTOR_SPEED;
    private final String MOTOR_NAME;
    private final Integer MINIMUM_ROTATIONS; //using object int to allow @Nullable anotation.
    private final Integer MAXIMUM_ROTAIONS;
    private DcMotor.Direction direction;
    private final boolean HAS_MINIMUM_LIMIT_SWITCH;
    private final boolean HAS_MAXIMUM_LIMIT_SWITCH;
    private final boolean HAS_ENCODER;
    private final boolean GO_TO_MIN_AT_INIT;
    private final boolean GO_TO_MAX_AT_INIT;
    private final boolean HOLD_POSITION_WHEN_STOPPED;

    private final Integer ADDITIONAL_ROTATIONS_TO_OVERRIDE_LIMIT_SWITCH;

    private DigitalChannel minimumLimitSwitch;
    private final String MINIMUM_LIMIT_SWITCH_NAME;

    private DigitalChannel maximumLimitSwitch;
    private final String MAXIMUM_LIMIT_SWITCH_NAME;

    LimitedDcMotorDrivenActuator(final String MOTOR_NAME, @Nullable final Integer MINIMUM_ROTATIONS,
                                 @Nullable final Integer MAXIMUM_ROTATIONS, DcMotor.Direction direction,
                                 final boolean HAS_MINIMUM_LIMIT_SWITCH,
                                 final boolean HAS_MAXIMUM_LIMIT_SWITCH,
                                 final boolean HAS_ENCODER,
                                 @Nullable final String MINIMUM_LIMIT_SWITCH_NAME,
                                 @Nullable final String MAXIMUM_LIMIT_SWITCH_NAME,
                                 @Nullable final Integer ADDITIONAL_ROTATIONS_TO_OVERRIDE_LIMIT_SWITCH,
                                 final boolean GO_TO_MIN_AT_INIT, final boolean GO_TO_MAX_AT_INIT,
                                 final boolean HOLD_POSITION_WHEN_STOPPED,
                                 final double INIT_MOTOR_SPEED) throws IllegalArgumentException {

        this.HAS_MINIMUM_LIMIT_SWITCH = HAS_MINIMUM_LIMIT_SWITCH;
        this.HAS_MAXIMUM_LIMIT_SWITCH = HAS_MAXIMUM_LIMIT_SWITCH;
        this.HAS_ENCODER = HAS_ENCODER;
        this.GO_TO_MIN_AT_INIT = GO_TO_MIN_AT_INIT;
        this.GO_TO_MAX_AT_INIT = GO_TO_MAX_AT_INIT;

        this.ADDITIONAL_ROTATIONS_TO_OVERRIDE_LIMIT_SWITCH = ADDITIONAL_ROTATIONS_TO_OVERRIDE_LIMIT_SWITCH;

        if(!HAS_ENCODER && !HAS_MAXIMUM_LIMIT_SWITCH && !HAS_MINIMUM_LIMIT_SWITCH)
            throw new IllegalArgumentException("Cannot limit motion without encoding or limit switch");

        if(GO_TO_MIN_AT_INIT && GO_TO_MAX_AT_INIT)
            throw new IllegalArgumentException("Cannot start at min and max");

        if(HAS_ENCODER && (MINIMUM_ROTATIONS == null || MAXIMUM_ROTATIONS == null))
            throw new IllegalArgumentException("Encoded actuators must specify min and max rotations regardless of min or max limit switch as fail safe");

        if(!HAS_ENCODER && (MAXIMUM_ROTATIONS != null || MINIMUM_ROTATIONS != null))
            throw new IllegalArgumentException("Cannot use max and min rotations without an encoder");

        if(!HAS_ENCODER && HOLD_POSITION_WHEN_STOPPED)
            throw new IllegalArgumentException("Cannot track position change to stop robot without encoder");

        if(ADDITIONAL_ROTATIONS_TO_OVERRIDE_LIMIT_SWITCH == null && HAS_ENCODER && (HAS_MAXIMUM_LIMIT_SWITCH || HAS_MINIMUM_LIMIT_SWITCH))
            throw new IllegalArgumentException("Must specify override if using encoder and limit switches");

        if(ADDITIONAL_ROTATIONS_TO_OVERRIDE_LIMIT_SWITCH != null && !HAS_ENCODER && (HAS_MAXIMUM_LIMIT_SWITCH || HAS_MINIMUM_LIMIT_SWITCH))
            throw new IllegalArgumentException("Cannot count rotations without encoder");

        if(ADDITIONAL_ROTATIONS_TO_OVERRIDE_LIMIT_SWITCH != null && HAS_ENCODER && !(HAS_MAXIMUM_LIMIT_SWITCH || HAS_MINIMUM_LIMIT_SWITCH))
            throw new IllegalArgumentException("No limit switch to override");

        if(!GO_TO_MIN_AT_INIT && !HAS_MINIMUM_LIMIT_SWITCH)
            throw new IllegalArgumentException("Cannot locate 0 position without minimum limit switch");


        this.MOTOR_NAME = MOTOR_NAME;
        this.MINIMUM_ROTATIONS = MINIMUM_ROTATIONS;
        this.MAXIMUM_ROTAIONS = MAXIMUM_ROTATIONS;
        this.direction = direction;

        this.MAXIMUM_LIMIT_SWITCH_NAME = MAXIMUM_LIMIT_SWITCH_NAME;
        this.MINIMUM_LIMIT_SWITCH_NAME = MINIMUM_LIMIT_SWITCH_NAME;

        this.INIT_MOTOR_SPEED = INIT_MOTOR_SPEED;

        this.HOLD_POSITION_WHEN_STOPPED = HOLD_POSITION_WHEN_STOPPED;
    }

    public void initHardware(HardwareMap ahwMap) {
        motor = ahwMap.get(DcMotor.class, MOTOR_NAME);
        motor.setDirection(direction);
        motor.setPower(0);

        if(HAS_MINIMUM_LIMIT_SWITCH){
            minimumLimitSwitch = ahwMap.get(DigitalChannel.class, MINIMUM_LIMIT_SWITCH_NAME);
            minimumLimitSwitch.setMode(DigitalChannel.Mode.INPUT);
        }

        if(HAS_MAXIMUM_LIMIT_SWITCH){
            maximumLimitSwitch = ahwMap.get(DigitalChannel.class, MAXIMUM_LIMIT_SWITCH_NAME);
            maximumLimitSwitch.setMode(DigitalChannel.Mode.INPUT);
        }

        if(HAS_ENCODER){
            if(HOLD_POSITION_WHEN_STOPPED)
                motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            else if (!HOLD_POSITION_WHEN_STOPPED)
                motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

            if(GO_TO_MIN_AT_INIT){
                if(!HAS_MINIMUM_LIMIT_SWITCH)
                {
                    motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);//assume user starts robot at zero positino
                    motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                }
                moveToMinPos(INIT_MOTOR_SPEED);

            }
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setPower(0); //just to be sure.
        }
        else {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        if(GO_TO_MIN_AT_INIT){
            moveToMinPos(INIT_MOTOR_SPEED);
        }
        if(GO_TO_MAX_AT_INIT){
            moveToMaxPos(INIT_MOTOR_SPEED);
        }

    }

    public int getCurrentPosition(){
        if (HAS_ENCODER){
            return motor.getCurrentPosition();
        }
        else return -1;

    }

    public void moveDegrees(double speed, int degrees){
        //figure out max rotations of motor by getting motor name
        moveRotations(speed, (int)((degrees/360)*motor.getMotorType().getTicksPerRev())); //TODO see how/if this works. Is rounding or truncating? effect?

    }

    public void moveRotations(double speed, @Nullable Integer rotations) throws IllegalArgumentException {
        int rotationTarget;

        if (rotations == null &&( !HAS_MAXIMUM_LIMIT_SWITCH || !HAS_MINIMUM_LIMIT_SWITCH ) && !HAS_ENCODER) {
            throw new IllegalArgumentException("Cannot limit motion without encoders at both limits if missing encoder.");
        }

        //basic physics. Speed is |velocity| (speed is always positive)
        if(rotations < 0)
            speed = -Math.abs(speed);
            //motor.setPower(-Math.abs(speed));
        else
            speed = Math.abs(speed);
           // motor.setPower(Math.abs(speed)); //direction set.


        //if((HAS_MINIMUM_LIMIT_SWITCH || (rotations == MINIMUM_ROTATIONS && HAS_MINIMUM_LIMIT_SWITCH && speed < 0))){
        if(HAS_MINIMUM_LIMIT_SWITCH && speed < 0){
            //speed = -Math.abs(speed);
            minimumLimitSwitch.setState(false);
            motor.setPower(speed);
            while (/*true*/!getMinimumLimitSwitchPressed() && speed <0){ //TODO figure out what's going on with getting limit switch states
                if(HAS_ENCODER){
                    if(motor.getCurrentPosition() < MINIMUM_ROTATIONS - ADDITIONAL_ROTATIONS_TO_OVERRIDE_LIMIT_SWITCH)  //play around with this threshold?
                        break;
                }
                /*
                if(!getMinimumLimitSwitchPressed())
                    break;
                 */
            } //wait for limit swutch to press. But break if passed the rotation limits (as a safety in case limit switch does not work)
            motor.setPower(0); //then don't forget to stop motor.
            if(HAS_ENCODER){
                motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);  //hopefully actuator will not fall too much in the time it resets encoder
                motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            motor.setPower(0); //then don't forget to stop motor.
            return;
        }

        //if (HAS_MAXIMUM_LIMIT_SWITCH || (rotations == MAXIMUM_ROTAIONS && HAS_MAXIMUM_LIMIT_SWITCH)) {  //default to these commands if limit switch. For maximum, exit method after. For minimum, it is possible to reset encoder, so no need to exit.
        if (HAS_MAXIMUM_LIMIT_SWITCH && speed > 0){
            motor.setPower(speed);
            while (!maximumLimitSwitch.getState() && speed > 0) {
                if(HAS_ENCODER){
                    if(motor.getCurrentPosition() > MAXIMUM_ROTAIONS + ADDITIONAL_ROTATIONS_TO_OVERRIDE_LIMIT_SWITCH)  //play around with this threshold?
                        break;
                }
            }
            motor.setPower(0);
            return;
        }

        if (HAS_ENCODER && rotations != null) {

            if (HAS_MINIMUM_LIMIT_SWITCH && motor.getCurrentPosition() < (0.5*(MAXIMUM_ROTAIONS-MINIMUM_ROTATIONS))) {  //resonably should be closer to 0 position. This also compensates for magnetic limit switches using one magnetic limit switch and a magnet at both ends
                if (minimumLimitSwitch.getState()) {
                    if (MINIMUM_ROTATIONS == 0){
                        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //set to zero. Unfortunately, I have found no other way to change the rotation value of encoders.
                        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    }
                    return;
                }
            }

            if (HAS_MAXIMUM_LIMIT_SWITCH && motor.getCurrentPosition() > (0.5*MAXIMUM_ROTAIONS-MINIMUM_ROTATIONS)) {  //account for magnetic limit switch. If rotaitons is bigger than midpoint, most likely upper limit.
                if (maximumLimitSwitch.getState()) {
                    return;
                }
            }
            if (motor.getCurrentPosition() >= MAXIMUM_ROTAIONS && (rotations > 0)) //abort if already erect
                return;
            if ((motor.getCurrentPosition() <= MINIMUM_ROTATIONS) && (rotations < 0)) //another reasons to abort
                return;
            rotationTarget = motor.getCurrentPosition() + rotations;
            motor.setTargetPosition(rotationTarget);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setPower(speed);
            while (motor.isBusy()) ;//wait for motor to moveRotations
            motor.setPower(0);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void moveToRotationCount(double speed, int rotationCount){
        moveRotations(speed, rotationCount - getCurrentPosition());//if pos is 5000, 5000-3000 = need to move -2000 to get to position
    }

    public void moveToMaxPos(double speed){
        moveRotations(Math.abs(speed), MAXIMUM_ROTAIONS-getCurrentPosition()); //get rotations left to move to maz
    }

    public void moveToMinPos(double speed){
        moveRotations(Math.abs(speed), MINIMUM_ROTATIONS-getCurrentPosition());//e.g. 0-300 = -300. Move -300 rotoations to get to 0.
    }

    public void teleOpMoveWithButtons(boolean moveToMaxPosButton, boolean moveToMinPosButton, double speed){
        //always test limit switches first. Best limit test as sense physical limit.
        if (moveToMaxPosButton || moveToMinPosButton) {
            if (HAS_MAXIMUM_LIMIT_SWITCH){
                if(!maximumLimitSwitch.getState() && moveToMaxPosButton){
                    speed = (Math.abs(speed));
                }
                else speed = 0;
            }

            if(HAS_MINIMUM_LIMIT_SWITCH){
                if(!minimumLimitSwitch.getState() && moveToMinPosButton){
                    speed = (-(Math.abs(speed)));
                }
                else speed = 0;
            }

            if (HAS_ENCODER){
                if((motor.getCurrentPosition() <= MAXIMUM_ROTAIONS) && moveToMaxPosButton){
                    speed = (Math.abs(speed));
                }
                else if ((motor.getCurrentPosition() >= MINIMUM_ROTATIONS) && moveToMinPosButton){
                    speed = (-(Math.abs(speed)));
                }
                else speed = 0;
            }
        }
        else speed = 0;

        motor.setPower(speed);
        //only set speed once

        //THANK YOU: https://www.vexforum.com/index.php/13453-robotc-bad-programming-habits/p1#p123070

        /*


        do {
            if(!moveToMaxPosButton || !moveToMinPosButton)
            {
                break;
            }
        }while (moveToMaxPosButton || moveToMinPosButton);


        while (moveToMaxPosButton || moveToMinPosButton){
            if(!moveToMaxPosButton || !moveToMinPosButton)
            {
                break;
            }

        } //wait for button to release

        //stop motor.
        motor.setPower(0);
        */
    }

    public void teleOpMoveWithJoystick(double joyStickDouble){ //TODO test this
        double speed = 0.0;
        joyStickDouble = -joyStickDouble;
        if (joyStickDouble != 0.0) { //TODO maybe put a tolerance
            if (HAS_MAXIMUM_LIMIT_SWITCH){
                if(!maximumLimitSwitch.getState() && joyStickDouble > 0.0){
                    speed = joyStickDouble;
                }
                else speed = 0;
            }

            if(HAS_MINIMUM_LIMIT_SWITCH){
                if(!minimumLimitSwitch.getState() && joyStickDouble < 0.0){
                    speed = joyStickDouble;
                }
                else speed = 0;
            }

            if (HAS_ENCODER){
                if((motor.getCurrentPosition() <= MAXIMUM_ROTAIONS) && joyStickDouble > 0.0){
                    speed = joyStickDouble;
                }
                else if ((motor.getCurrentPosition() >= MINIMUM_ROTATIONS) && joyStickDouble < 0.0){
                    speed = joyStickDouble;
                }
                else speed = 0;
            }
        }
        else speed = 0;

        motor.setPower(speed);
    }

    public void teleOpMoveToMinPos(boolean moveToMinPosButton, double speed){
        if (moveToMinPosButton)
            moveToMinPos(speed);
    }

    public void teleOpMoveToMaxPos(boolean moveToMaxPosButton, double speed){
        if(moveToMaxPosButton)
            moveToMaxPos(speed);
    }

    public void setBraking(boolean trueForOneFalseForOff) throws IllegalArgumentException{
        if(!HAS_ENCODER)
            throw new IllegalArgumentException("Cannot use this feature without encoder");
        if(trueForOneFalseForOff)
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        else if(!trueForOneFalseForOff)
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public String getLimitSwitchStates(){
        String report = null;
        if(HAS_MINIMUM_LIMIT_SWITCH)
            report = "MINIMUM_LIMIT_SWITCH" + minimumLimitSwitch.getState();
        else if(HAS_MAXIMUM_LIMIT_SWITCH)
            report += "\nMAXIMUM LIMIT SWITCH" + maximumLimitSwitch.getState();
        else
            report = "-1";
        return report;
    }

    private boolean getMinimumLimitSwitchPressed(){
        return !minimumLimitSwitch.getState(); //inverts so true. //TODO figure this out
    }

    public boolean getIsBusy(){
        return motor.isBusy();
    }

    public Integer getMINIMUM_ROTATIONS() {
        return MINIMUM_ROTATIONS;
    }

    public Integer getMAXIMUM_ROTAIONS() {
        return MAXIMUM_ROTAIONS;
    }
}
