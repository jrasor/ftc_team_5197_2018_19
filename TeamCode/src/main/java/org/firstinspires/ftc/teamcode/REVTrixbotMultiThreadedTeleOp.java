/**
 * Meet ILT TeleOp code
 * @Author Lorenzo Pedroza
 * Date 2/03/19
 *
 */


package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "REVTrixbot Monrovia ILT TeleOp", group = "ILT")
//@Disabled
public class REVTrixbotMultiThreadedTeleOp extends OpMode {

    REVTrixbot robot = new REVTrixbot();

    //TODO check concern that methods with while loops will stop if thread is interrupted (shouldn't be a problem per https://www.tutorialspoint.com/java/lang/thread_isinterrupted.htm)

    volatile private static String driveTrainStatus = "Drivetrain Status";
    volatile private static String mineralArmStatus = "Mineral Arm Status";
    volatile private static String mineralLifterArmRaiserStatus = "Arm Raiser Status";
    volatile private static String mineralLifterArmExtenderStatus = "Arm Extender Status";
    volatile private static String mineralLifterWristStatus = "Arm Wrist Status";
    volatile private static String mineralLifterGripperStatus = "Arm Gripper Status";

    private static final String EXTENDER_LENGTH_LOCKED = "Length Locked";
    private static double extenderLenght = 22.0; //TODO measure unextedned lenght
    private static double lockedLenght = 0.0; //found in in method lockLenght()
    private static final double ROBOT_EXTENDER_HEIGHT = 17.5; //TODO measure
    private static final int LINEARSLIDE_COUNTS_PER_INCH = 100;
    private static final int OPERABLE_ROTATIONS_FOR_LOCK = 5000; //TODO adjsut this too
    private static boolean MACROS_ENABLED = true;

    private static final String INITIALIZED = "INITIALIZED";
    private static final String RUNNING = "Running";
    private static final String MOVING_TO_MINERAL_DEPOSIT_POS = "Moving to Mineral Deposit Pos";
    private static final String MOVING_TO_MINERAL_COLLECT_POS = "Moving to Mineral Collect Pos";


    final int LA_ARM_LIFTER_MINERAL_DEPOSIT_POS = 3100; //TODO figure out this value
    final int LA_ARM_DEPOSIT_POS = 1300; //TODO and this value

    final int LA_ARM_LIFTER_MINERAL_COLLECT_POS = 5000; //TODO figure out this value
    final int LA_ARM_COLLECT_POS = 1300; //TODO and this value

    public void macroToggle(boolean button){
        if(button)
            MACROS_ENABLED = false;
    }

    @Override
    public void init() {
        MACROS_ENABLED = true;
        robot.threadDT = new REVTrixbot.REVTrixbotMTDrivetrain(){
            @Override
            public void run() {
                super.run();
                driveTrainStatus = RUNNING;
                while(!isInterrupted()){
                    //teleOpTankDrive(gamepad1);
                    teleOpArcadeDrive(gamepad1, F310JoystickInputNames.Joysticks.LEFT_STICK);
                }
            }
        };
        robot.threadDT.initHardware(hardwareMap);
        driveTrainStatus = INITIALIZED;

        robot.threadMineralLifter = new REVTrixbot.REVTrixbotMTMineralLifter(){
            private static final int LIFTER_PARALLEL_TO_ROBOT_POSITION = 100;
            @Override
            public void teleOpFullyStowMTMineralLifter(double laArmSpeed, double laArmLifterSpeed, boolean button) { //macro
                if(button)
                {
                    mineralArmStatus = "Fully Stowing Mineral Arm";
                    mineralLifterArmRaiserStatus = "Stowing";
                    mineralLifterArmExtenderStatus = "Stowing";
                    mineralLifterWristStatus = "Stowing";
                    //mineralLifterGripperStatus = "Stowing";
                    super.teleOpFullyStowMTMineralLifter(laArmSpeed, laArmLifterSpeed, button);
                    mineralArmStatus = RUNNING;
                    mineralLifterArmRaiserStatus = RUNNING;
                    mineralLifterArmExtenderStatus = RUNNING;
                    mineralLifterWristStatus = RUNNING;
                    //mineralLifterGripperStatus = RUNNING;
                }
            }


            @Override
            public void run() {
                super.run();
                //Declare all running systems
                mineralArmStatus = RUNNING;
                mineralLifterGripperStatus = RUNNING;
                mineralLifterWristStatus = RUNNING;
                while(!isInterrupted()){
                    //teleOpFullyStowMTMineralLifter(0.3, 0.3, gamepad2.y);
                    //getInMineralCargoBayDropPosition(gamepad2.dpad_up);
                   // getInMineralCraterCollectPosition(gamepad2.dpad_down);
                    //teleOpSingleButtonGrip(gamepad2.a);

                    teleOpRotateWrist(gamepad2.y && mineralLifterWristStatus == RUNNING, gamepad2.a && mineralLifterWristStatus == RUNNING);
                    //teleOpRotateWristWithGamepadTriggers(gamepad2); //Doesn't work too well
                    //teleOpGrip(gamepad2.a, gamepad2.b);
                    keepServoLevelToGround(gamepad2.right_bumper && mineralLifterWristStatus == RUNNING); //TODO test and refine this method

                }
            }
        };

        robot.threadMineralLifter.threadedLinearActuatorArm = new REVTrixbot.REVTrixbotMTMineralLifter.ThreadedLinearActuatorArm(){
            @Override
            public void teleOpMoveWithJoystick(double joyStickDouble) {
                switch (mineralLifterArmExtenderStatus) {
                    case RUNNING:
                        super.teleOpMoveWithJoystick(joyStickDouble);
                        break;
                    case MOVING_TO_MINERAL_COLLECT_POS:
                        moveToRotationCount(0.7, LA_ARM_COLLECT_POS); //go a little slower to prevent collisions by allowing the lifter to be ahead
                        mineralLifterArmExtenderStatus = RUNNING;
                        break;
                    case MOVING_TO_MINERAL_DEPOSIT_POS:
                        moveToRotationCount(0.7, LA_ARM_DEPOSIT_POS);
                        mineralLifterArmExtenderStatus = RUNNING;
                        break;
                    case EXTENDER_LENGTH_LOCKED:
                        double lockedPositionInches = (lockedLenght/Math.sin((double)(getMAXIMUM_ROTAIONS()-getCurrentPosition())*0.5)); //TODO find rotations per degree (convert rotations to angle in radians)
                        //move to satisfy formulas and stuff(I'm working on it)
                        if(getCurrentPosition()<OPERABLE_ROTATIONS_FOR_LOCK)
                        {
                            mineralLifterArmExtenderStatus = RUNNING;
                            break;
                        }
                        if(gamepad1.right_stick_y != 0.0)
                            moveToRotationCount(0.75, (int)(lockedPositionInches-22.0)*LINEARSLIDE_COUNTS_PER_INCH);
                        break;
                }
            }

            public void lockLength(boolean lockButtonToggle){
                if(lockButtonToggle)
                {
                    if(mineralLifterArmExtenderStatus == RUNNING) //not locked and running
                    {
                        mineralLifterArmExtenderStatus = EXTENDER_LENGTH_LOCKED; //then lock it
                        extenderLenght = (double)(getCurrentPosition())/(double)(LINEARSLIDE_COUNTS_PER_INCH) + 22.0; //TODO adjust scale and find base he
                        lockedLenght = Math.sqrt(Math.pow(extenderLenght, 2.0) - Math.pow(ROBOT_EXTENDER_HEIGHT, 2.0));
                    }

                    else if(mineralLifterArmExtenderStatus == EXTENDER_LENGTH_LOCKED) //locked
                        mineralLifterArmExtenderStatus = RUNNING; //unlock it
                }
            }




            @Override
            public void run() {
                super.run();
                mineralLifterArmExtenderStatus = RUNNING;
                while (!isInterrupted()){
                    teleOpMoveWithJoystick(gamepad2.left_stick_y); //for Clinston Zeng's preference
                    //lockLength(gamepad2.left_bumper);//TODO test this 2nd
                    //teleOpMoveWithButtons(gamepad2.a, gamepad2.b, 1);
                }
            }
        };

        robot.threadMineralLifter.threadedArmLifter = new REVTrixbot.REVTrixbotMTMineralLifter.ThreadedArmLifter(){
            public void getInMineralCargoBayDropPosition(boolean button){ //set string flags to start a macro
                if(button && MACROS_ENABLED) //only do once gripper closed with mineral. May need to add tolerance
                {
                    mineralLifterArmExtenderStatus = MOVING_TO_MINERAL_DEPOSIT_POS;
                    mineralLifterArmRaiserStatus = MOVING_TO_MINERAL_DEPOSIT_POS;
                }
            }

            public void getInMineralCraterCollectPosition(boolean button){////set string flags to start a macro
                if(button && MACROS_ENABLED)
                {
                    mineralLifterArmExtenderStatus = MOVING_TO_MINERAL_COLLECT_POS;
                    mineralLifterArmRaiserStatus = MOVING_TO_MINERAL_COLLECT_POS;
                }
            }

            @Override
            public void teleOpMoveToHighestPosition(double speed, boolean button) {
                if(button && MACROS_ENABLED)
                {
                    mineralLifterArmRaiserStatus = "Moving to highest position";
                    super.teleOpMoveToHighestPosition(speed, button);
                    mineralLifterArmRaiserStatus = RUNNING;
                }
            }

            @Override
            public void teleOpMoveWithJoystick(double joyStickDouble) {
                switch (mineralLifterArmRaiserStatus) {
                    case RUNNING:
                        super.teleOpMoveWithJoystick(joyStickDouble);
                        break;
                    case MOVING_TO_MINERAL_COLLECT_POS:
                        moveToRotationCount(1.0, LA_ARM_LIFTER_MINERAL_COLLECT_POS);
                        mineralLifterArmRaiserStatus = RUNNING;
                        break;
                    case MOVING_TO_MINERAL_DEPOSIT_POS:
                        moveToRotationCount(1.0, LA_ARM_LIFTER_MINERAL_DEPOSIT_POS);
                        mineralLifterArmRaiserStatus = RUNNING;
                        break;
                }
            }

            @Override
            public void run() {
                super.run();
                mineralLifterArmRaiserStatus = RUNNING;
                while (!isInterrupted()){
                    //teleOpMoveToHighestPosition(0.3, gamepad1.y);
                    //teleOpMoveWithButtons(gamepad2.dpad_up, gamepad2.dpad_down, 1.0);
                    teleOpMoveWithJoystick(gamepad1.right_stick_y);
                    getInMineralCargoBayDropPosition(gamepad1.dpad_up); //TODO test and tune these macros 1st
                    getInMineralCraterCollectPosition(gamepad1.dpad_down);
                }
            }
        };

        robot.threadMineralLifter.initHardware(hardwareMap);
        mineralArmStatus = INITIALIZED;
        mineralLifterWristStatus = INITIALIZED;
        mineralLifterGripperStatus = INITIALIZED;
        mineralLifterArmExtenderStatus = INITIALIZED;
        mineralLifterArmRaiserStatus = INITIALIZED;

    }

    @Override
    public void start() {
        super.start();
        robot.threadMineralLifter.start();
        robot.threadDT.start();
        resetStartTime();
    }

    @Override
    public void internalPostInitLoop() {
        super.internalPostInitLoop();
        telemetry.addData("Macros Enabled", MACROS_ENABLED);
        telemetry.addData("Drivetrain Status", driveTrainStatus);
        telemetry.addData("Mineral Lifter Status", mineralArmStatus);
        telemetry.addData("   Raiser Status", mineralLifterArmRaiserStatus);
        telemetry.addData("   Extender Status", mineralLifterArmExtenderStatus);
        telemetry.addData("   Wrist Status", mineralLifterWristStatus);
        //telemetry.addData("   Gripper Status", mineralLifterGripperStatus);
    }

    @Override
    public void loop() {
       //robot.threadMineralLifter.teleOpSingleButtonGrip(gamepad2.a);
       //robot.threadMineralLifter.teleOpRotateWristWithGamepadTriggers(gamepad2.y, gamepad2.b);
        macroToggle(gamepad1.right_bumper || gamepad2.left_bumper);
    }

    @Override
    public void internalPostLoop() {
        super.internalPostLoop();
        telemetry.addData("Macros Enabled", MACROS_ENABLED);
        telemetry.addData("Drivetrain Status", driveTrainStatus);
        telemetry.addData("Mineral Lifter Status", mineralArmStatus);
        telemetry.addData("   Raiser Status", mineralLifterArmRaiserStatus);
        telemetry.addData("   Extender Status", mineralLifterArmExtenderStatus);
        telemetry.addData("   Wrist Status", mineralLifterWristStatus);
        //telemetry.addData("   Gripper Status", mineralLifterGripperStatus);
    }

    @Override
    public void stop() {
        super.stop();
        robot.threadMineralLifter.interrupt();
        robot.threadDT.interrupt();
    }
}
