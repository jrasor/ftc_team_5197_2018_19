/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single
 * robot, our competition REVTrixbot. It will compete in 2018-2019 FTC game
 * "Rover Ruckus".
 *
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot"
 * for usage examples easily converted to run on a REVTrixbot.
 *
 * This robot class operates a 4-wheel drive pushbot with a linear slide for
 * latching onto the Lander, swappable claws to grab Minerals, and OpenCV for
 * Mineral color discrimination.
 *
 * Version history
 * ======= ======
 * v 0.1    10/11/18 jmr primitive version, just enough to test the drive train.
 *          No class hierarchy, no initialization or run mode methods. Yet.
 *
 * v 0.2    10/28/18 Use of modular code for four wheel drivetrain.
 *
 * v 0.3    (In development) REVTrix has one port controlling each side of the DT now. Therefore, it
 *           now uses a TwoWheelDriveTrain although each of the four wheels are powered. Lifter code
 *
 * v 0.3.5    Deposit team identifier.
 *
 * v 0.4.0    Updated gold detector
 *
 * v0.8.0     Multithreading Functionality and Arm Code (in progress)
 */

public class REVTrixbot extends GenericFTCRobot
{
    // REVTrixbot specific measurements
    // ** to do: calibration.
    private static final double     COUNTS_PER_MOTOR_REV    = 288 ;
    private static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;

    // REVTrixbot specific drive train members.
    // ** to do: check these for REVTrixbot dimensions.
    private static final double     WHEEL_DIAMETER_INCHES   = 3.5 ; // 90mm Traction Wheel
    private static final double     DRIVE_WHEEL_SEPARATION  = 15.0 ;
    private static final DcMotor.RunMode RUNMODE = DcMotor.RunMode.RUN_USING_ENCODER; //encoder cables installed 10/27/18

    private static final String REVTRIXBOT_LA_ARM_LIFTER_MOTOR_NAME = "EH2motor0";
    private static final int REVTRIXBOT_LA_ARM_LIFTER_STOWED_ROTATIONS = 0;
    private static final int REVTRIXBOT_LA_ARM_LIFTER_ERECT_ROTATIONS = 6125; //Was 6225
    private static final double FRACTION_OF_MAX_ARM_LIFTER_ROTATIONS = (((double)REVTRIXBOT_LA_ARM_LIFTER_ERECT_ROTATIONS)*2.0)/3.0; //adjust the divisor for adjusting range of collector being level to ground

    private static final String REVTRIXBOT_LA_MOTOR_NAME = "EH2motor1";
    private static final int REVTRIXBOT_LA_RETRACTED_ROTATIONS = 0;
    private static final int REVTRIXBOT_LA_EXTENDED_ROTATIONS = 1300;

    private static final String REVTRIXBOT_GRIPPER_SERVO_NAME = "EH2servo3";
    private static final double REVTRIXBOT_GRIPPER_CLOSED = 0.1;
    private static final double REVTRIXBOT_GRIPPER_OPEN = 0.9;

    private static final String REVTRIXBOT_GRIPPER_WRIST_NAME = "EH2servo2";
    private static final double REVTRIXBOT_GRIPPER_WRIST_MAX_UP_POS = 0.0;
    private static final double REVTRIXBOT_GRIPPER_WRIST_INIT_POS = 0.8;
    private static final double REVTRIXBOT_GRIPPER_WRIST_MAX_DOWN_POS = 1.0;

    private static final String REVTRIXBOT_TEAM_IDENTIFIER_DEPOSITOR_SERVO_NAME = "EH2servo5";
    private static final double REVTRIXBOT_TEAM_IDENTIFIER_DEPOSITOR_INIT_POS = 0.7; //TODO adjust see if direction is reversible
    private static final double REVTRIXBOT_TEAM_IDENTIFIER_DEPOSITOR_DEPOSIT_POS = 0.2; //TODO adjust

    REVTrixbot(){
        super.init();
    }

    //old objects left for compatibility with older code not using multithreading/instantiating in the opmode. IMPORTANT: DO NOT ERASE THESE OBJECTS
    FourWheelDriveTrain dt = new FourWheelDriveTrain(COUNTS_PER_MOTOR_REV, DRIVE_GEAR_REDUCTION,
            WHEEL_DIAMETER_INCHES, DRIVE_WHEEL_SEPARATION, RUNMODE, "EH1motor0", "EH1motor1",
            "EH1motor2", "EH1motor3");

    TeamIdenfifierDepositer idenfierFor5197Depositer = new TeamIdenfifierDepositer(REVTRIXBOT_TEAM_IDENTIFIER_DEPOSITOR_INIT_POS,
            REVTRIXBOT_TEAM_IDENTIFIER_DEPOSITOR_DEPOSIT_POS, REVTRIXBOT_TEAM_IDENTIFIER_DEPOSITOR_SERVO_NAME); //moveRotations to 180 at initHardware. Then to close to

    MineralPushingPaddles revTrixbotMineralPaddles = new MineralPushingPaddles(0.0, 0.0, 0.4, "EH1servo3", "EH1servo4");

    //Multithreadable object references. Instantiate and define their behaviours in the opmode
    REVTrixbotMTDrivetrain threadDT; //instantiate in opmode, where run method is defined

    REVTrixbotMTMineralLifter threadMineralLifter;

    GoldMineralDetector_2 goldLocator;

    Thread goldLocationUpdater; //need to use Thread as GoldLocator is Runnable since it cannot be subclassed

    REVTrixbotMTTeamIdentifierDepositor threadTeamIdentifierDepositor;


    /*

    LimitedDcMotorDrivenActuator roverRuckusRevTrixBotLift = new LimitedDcMotorDrivenActuator("EH2motor1",  //Clockwise rotation is up
            0, 3500, DcMotorSimple.Direction.FORWARD, true,  //TODO add a "tolerance" encoder counts value to allow a few encoders count off. Of course limit switches are always better.
            false, true, "EH1touchsensor1", null,
            30,
            true, false, true, 0.01); //TODO maybe thorw IllegalArgument exception for going to Min or Max without limit switch. Need to see if rotations being counted before runtime.
     */
    MineralLifter revTrixBotMineralArm = new MineralLifter(REVTRIXBOT_GRIPPER_CLOSED, REVTRIXBOT_GRIPPER_OPEN,
            REVTRIXBOT_LA_ARM_LIFTER_STOWED_ROTATIONS, REVTRIXBOT_LA_ARM_LIFTER_ERECT_ROTATIONS, REVTRIXBOT_LA_RETRACTED_ROTATIONS,
            REVTRIXBOT_LA_EXTENDED_ROTATIONS, REVTRIXBOT_GRIPPER_SERVO_NAME, REVTRIXBOT_GRIPPER_WRIST_NAME, REVTRIXBOT_LA_ARM_LIFTER_MOTOR_NAME,
            REVTRIXBOT_LA_MOTOR_NAME); //Not ready.


    public static class REVTrixbotMTDrivetrain extends FourWheelDriveTrain{  //TODO: Temporaily giving it this name to ease transition to multithreaded code.
        REVTrixbotMTDrivetrain(){
            super(REVTrixbot.COUNTS_PER_MOTOR_REV, REVTrixbot.DRIVE_GEAR_REDUCTION,
                    REVTrixbot.WHEEL_DIAMETER_INCHES, REVTrixbot.DRIVE_WHEEL_SEPARATION,
                    REVTrixbot.RUNMODE, "EH1motor0",
                    "EH1motor1", "EH1motor2",
                    "EH1motor3");
        }
    }

    public static class REVTrixbotMTMineralLifter extends MineralLifter{ //wrapper so parameters don't need to be inputted again in opmodes
        REVTrixbotMTMineralLifter(){
            super(REVTRIXBOT_GRIPPER_CLOSED, REVTRIXBOT_GRIPPER_OPEN,
                    REVTRIXBOT_LA_ARM_LIFTER_STOWED_ROTATIONS, REVTRIXBOT_LA_ARM_LIFTER_ERECT_ROTATIONS, REVTRIXBOT_LA_RETRACTED_ROTATIONS,
                    REVTRIXBOT_LA_EXTENDED_ROTATIONS, REVTRIXBOT_GRIPPER_SERVO_NAME, REVTRIXBOT_GRIPPER_WRIST_NAME,
                    REVTRIXBOT_LA_ARM_LIFTER_MOTOR_NAME, REVTRIXBOT_LA_MOTOR_NAME);
        }

        ThreadedArmLifter threadedArmLifter;
        ThreadedLinearActuatorArm threadedLinearActuatorArm;

        @Override
        public synchronized void start() {
            super.start();
            threadedArmLifter.start();
            threadedLinearActuatorArm.start();
        }

        @Override
        public void interrupt() {  //probably more professional way, but for now, this interrupts the entire MineralLifterSystem
            super.interrupt();
            threadedArmLifter.interrupt();
            threadedLinearActuatorArm.interrupt();
        }

        @Override
        public void initHardware(HardwareMap ahwMap) {
            threadedArmLifter.initHardware(ahwMap);
            threadedLinearActuatorArm.initHardware(ahwMap);
            gripper = ahwMap.get(Servo.class, GRIPPER_SERVO_NAME);
            gripper_wrist = ahwMap.get(Servo.class, GRIPPER_WRIST_NAME);
            gripper_wrist.setPosition(REVTRIXBOT_GRIPPER_WRIST_INIT_POS);
        }

        public void fullyStowMTMineralLifter(double laArmSpeed, double laArmLifterSpeed){ //macro only available in threaded version as while loops are not safe in linear code.
            //TODO move wrist to safe position in this line
            gripper.setPosition(GRIPPER_CLOSED);
            threadedArmLifter.moveToMinPos(laArmSpeed); //do first to prevent collisions with robot base
            threadedLinearActuatorArm.moveToMaxPos(laArmLifterSpeed);
        }

        public void teleOpFullyStowMTMineralLifter(double laArmSpeed, double laArmLifterSpeed, boolean button){
            if(button)
                fullyStowMTMineralLifter(laArmSpeed, laArmLifterSpeed);
        }

        public void keepServoLevelToGround(boolean overrideButton){ //TODO test this method

            double threadedArmLifterPosRatio = (((double) threadedArmLifter.getCurrentPosition())- FRACTION_OF_MAX_ARM_LIFTER_ROTATIONS)/ FRACTION_OF_MAX_ARM_LIFTER_ROTATIONS;
            if((double)threadedArmLifter.getCurrentPosition() > FRACTION_OF_MAX_ARM_LIFTER_ROTATIONS && !overrideButton)
                gripper_wrist.setPosition(threadedArmLifterPosRatio+0.6); //1.30


            /*
            if(!overrideButton)
            {
                if((laArmLifter.getCurrentPosition() >= laArmLifter.getMAXIMUM_ROTAIONS()/2 ||
                        laArmLifter.getCurrentPosition() <= laArmLifter.getMAXIMUM_ROTAIONS()/7)) //keep level when not in the range of dumping minerals
                    gripper_wrist.setPosition(levelServoPos);
            }
            */
        }






        public static class ThreadedArmLifter extends LimitedDcMotorDrivenActuator{
            private static final int HIGHEST_POSITION = 3000;

            ThreadedArmLifter(){
                super(REVTRIXBOT_LA_ARM_LIFTER_MOTOR_NAME,
                        REVTRIXBOT_LA_ARM_LIFTER_STOWED_ROTATIONS, REVTRIXBOT_LA_ARM_LIFTER_ERECT_ROTATIONS, DcMotorSimple.Direction.FORWARD,
                        false, false, true, null,
                        null, null,
                        true, false, true,0.0);
            }


            public void moveToHighestPosition(double speed){
                moveToRotationCount(speed, HIGHEST_POSITION);
                //if pos is 2000, 3000-2000 = +1000 to get to middle positin
            }

            public void teleOpMoveToHighestPosition(double speed, boolean button){
                if(button) //TODO make sure newest called methods is acted on actuator and no crazy stuff happens when new called method interrupts current one
                    moveToHighestPosition(speed);
            }


        }

        public static class ThreadedLinearActuatorArm extends LimitedDcMotorDrivenActuator{
            public ThreadedLinearActuatorArm(){
                super(REVTRIXBOT_LA_MOTOR_NAME, REVTRIXBOT_LA_RETRACTED_ROTATIONS, REVTRIXBOT_LA_EXTENDED_ROTATIONS, DcMotorSimple.Direction.FORWARD, false,
                        false, true, null, null,
                        null,
                        true, false, true,0.0);
            }
        }

    }

    public static class MineralLifter extends Thread implements FTCModularizableSystems{ //nested since it is technically not modularizable
        protected Servo gripper = null;
        protected Servo gripper_wrist = null;
        protected final double GRIPPER_CLOSED;
        protected final double GRIPPER_OPEN;
        protected final String GRIPPER_SERVO_NAME;
        protected final String GRIPPER_WRIST_NAME;
        protected final double LA_ARM_LIFTER_STOWED_ROTATIONS;
        protected final double LA_ARM_LIFTER_ERECT_ROTATIONS;
        protected final double LA_RETRACTED_ROTATIONS;
        protected final double LA_EXTENDED_ROTATIONS;

        LimitedDcMotorDrivenActuator laArmLifter;
        LimitedDcMotorDrivenActuator laArm;

        MineralLifter(final double GRIPPER_CLOSED, final double GRIPPER_OPEN, final int LA_ARM_LIFTER_STOWED_ROTATIONS,
                      final int LA_ARM_LIFTER_ERECT_ROTATIONS, final int LA_RETRACTED_ROTATIONS, final int LA_EXTENDED_ROTATIONS,
                      final String GRIPPER_SERVO_NAME, final String GRIPPER_WRIST_NAME, final String LA_ARM_LIFTER_MOTOR_NAME, final String LA_MOTOR_NAME){ //TODO Maybe Multithread Lifter and LA so they run simultaneosly
            this.GRIPPER_CLOSED = GRIPPER_CLOSED;
            this.GRIPPER_OPEN = GRIPPER_OPEN;
            this.GRIPPER_SERVO_NAME = GRIPPER_SERVO_NAME;
            this.GRIPPER_WRIST_NAME = GRIPPER_WRIST_NAME;
            this.LA_ARM_LIFTER_STOWED_ROTATIONS = LA_ARM_LIFTER_STOWED_ROTATIONS;
            this.LA_ARM_LIFTER_ERECT_ROTATIONS = LA_ARM_LIFTER_ERECT_ROTATIONS;
            this.LA_RETRACTED_ROTATIONS = LA_RETRACTED_ROTATIONS;
            this.LA_EXTENDED_ROTATIONS = LA_EXTENDED_ROTATIONS;


            laArmLifter = new LimitedDcMotorDrivenActuator(LA_ARM_LIFTER_MOTOR_NAME,
                    LA_ARM_LIFTER_STOWED_ROTATIONS, LA_ARM_LIFTER_ERECT_ROTATIONS, DcMotorSimple.Direction.FORWARD,
                    false, false, true, null,
                    null, null,
                    true, false, true,0.0);

            laArm = new LimitedDcMotorDrivenActuator(LA_MOTOR_NAME,
                    LA_RETRACTED_ROTATIONS, LA_EXTENDED_ROTATIONS, DcMotorSimple.Direction.FORWARD, false,
                    false, true, null, null,
                    null,
                    true, false, true,0.0);
        }

        public void initHardware(HardwareMap ahwMap){
            gripper = ahwMap.get(Servo.class, GRIPPER_SERVO_NAME);
            gripper_wrist = ahwMap.get(Servo.class, GRIPPER_WRIST_NAME);
            //gripper_wrist = ahwMap.get(Servo.class, );
            closeGripper();

            laArmLifter.initHardware(ahwMap);
            laArm.initHardware(ahwMap);

        }

        public void teleOpGrip(boolean gripButton, boolean openButton){ //preferably a trigger
            if(gripButton)
                closeGripper();
            if(openButton)
                openGripper();
        }

        public void teleOpSingleButtonGrip(boolean gripButton){
            if(gripButton)
            {
                if(gripper.getPosition() == GRIPPER_OPEN)
                    gripper.setPosition(GRIPPER_CLOSED);
                else
                    gripper.setPosition(GRIPPER_OPEN);
            }
        }

        public void teleOpRotateWrist(boolean moveUpButton, boolean moveDownButton){
            if(moveUpButton && gripper_wrist.getPosition() < REVTRIXBOT_GRIPPER_WRIST_MAX_DOWN_POS)
                gripper_wrist.setPosition(gripper_wrist.getPosition()+0.005);
            else if(moveDownButton && gripper_wrist.getPosition() > REVTRIXBOT_GRIPPER_WRIST_MAX_UP_POS)
                gripper_wrist.setPosition(gripper_wrist.getPosition()-0.005);
        }

        public void teleOpRotateWristWithGamepadTriggers(Gamepad weaponsOfficerGamepead){ //needs to be properly tested. Not too responsive enough right now
            if (weaponsOfficerGamepead.left_trigger != 0.0 || weaponsOfficerGamepead.right_trigger != 0.0) //don't write to servos to save resources when triggers not pressed
                if(gripper_wrist.getPosition() < REVTRIXBOT_GRIPPER_WRIST_MAX_UP_POS)
                   gripper_wrist.setPosition(gripper_wrist.getPosition()+weaponsOfficerGamepead.left_trigger);
                else if (gripper_wrist.getPosition() > REVTRIXBOT_GRIPPER_WRIST_MAX_DOWN_POS)
                    gripper_wrist.setPosition(gripper_wrist.getPosition()-weaponsOfficerGamepead.right_trigger);
        }




        /*
        public void teleOpGripArmUpDown (boolean upButton, boolean downButton){
            if (upButton)
                upGripperArm();
            if (downButton)
                downGripperArm();
        }
        */

        public void openGripper(){
            gripper.setPosition(GRIPPER_OPEN);
        }

        public void closeGripper(){
            gripper.setPosition(GRIPPER_CLOSED);
        }

        //TODO figure out wrist movement


       // public void upGripperArm(){ gripper_wrist.setPosition( UP_AND_DOWN + 1.0 ); } TODO study this

       // public void downGripperArm(){ gripper_wrist.setPosition( UP_AND_DOWN - 1.0 );} TODO study this


    }

    public static class TeamIdenfifierDepositer extends Thread implements FTCModularizableSystems{ //does not really need to extend thread, but being consistent
        private Servo glypgDepositServo = null;
        private final double INIT_POS;
        private final double DEPOSIT_POS;
        private final String SERVO_NAME;

        TeamIdenfifierDepositer(final double INIT_POS, final double DEPOSIT_POS, final String SERVO_NAME){
            this.INIT_POS = INIT_POS;
            this.DEPOSIT_POS = DEPOSIT_POS;
            this.SERVO_NAME = SERVO_NAME;
        }

        public void initHardware(HardwareMap ahwMap) {
            glypgDepositServo = ahwMap.get(Servo.class, SERVO_NAME);
            glypgDepositServo.setPosition(INIT_POS);
        }

        public void depositTeamIdentifier(){
            glypgDepositServo.setPosition(DEPOSIT_POS);
        }
    }

    public static class REVTrixbotMTTeamIdentifierDepositor extends TeamIdenfifierDepositer{ //wrapper to make it easier to program in opmode
        REVTrixbotMTTeamIdentifierDepositor(){
            super(REVTRIXBOT_TEAM_IDENTIFIER_DEPOSITOR_INIT_POS, REVTRIXBOT_TEAM_IDENTIFIER_DEPOSITOR_DEPOSIT_POS, REVTRIXBOT_TEAM_IDENTIFIER_DEPOSITOR_SERVO_NAME);
        }
    }

    //Actuator that was going to be used but was never used
    public static class MineralPushingPaddles extends Thread implements FTCModularizableSystems{
        private Servo leftPaddle = null;
        private Servo rightPaddle = null;
        private final double INIT_POS;
        private final double RETRACTED_POS;
        private final double DEPLOYED_POS;
        private final String LEFT_SERVO_NAME;
        private final String RIGHT_SERVO_NAME;
        private boolean deployRetractButtonWasPressed = false;

        MineralPushingPaddles(final double INIT_POS, final double RETRACTED_POS, final double DEPLOYED_POS,
                              final String LEFT_SERVO_NAME, final String RIGHT_SERVO_NAME){
            this.INIT_POS = INIT_POS;
            this.RETRACTED_POS = RETRACTED_POS;
            this.DEPLOYED_POS = DEPLOYED_POS;
            this.LEFT_SERVO_NAME = LEFT_SERVO_NAME;
            this.RIGHT_SERVO_NAME = RIGHT_SERVO_NAME;
        }

        @Override
        public void initHardware(HardwareMap ahwMap) {
            leftPaddle = ahwMap.get(Servo.class, LEFT_SERVO_NAME);
            rightPaddle = ahwMap.get(Servo.class, RIGHT_SERVO_NAME);
            leftPaddle.setPosition(INIT_POS);
            rightPaddle.setPosition(INIT_POS);
        }

        public void teleOpDeployRetractPaddles(boolean deployRetractButton){
            if(deployRetractButton){
                leftPaddle.setPosition(DEPLOYED_POS);
                rightPaddle.setPosition(DEPLOYED_POS);
                deployRetractButtonWasPressed = true;
            }
            else if (deployRetractButtonWasPressed){
                leftPaddle.setPosition(RETRACTED_POS);
                rightPaddle.setPosition(RETRACTED_POS);
                deployRetractButtonWasPressed = false; //reset. Now
            }
        }
    }
}
