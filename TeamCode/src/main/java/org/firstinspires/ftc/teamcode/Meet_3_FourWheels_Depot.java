/**
 * Rover Ruckus Meet 3 Code
 * This code looks different in structure from previous meets because it now utilizes multithreading
 * for each system in the robot. This enables the safe use of while loops, henceforth enabling the
 * use of limit switches.
 *
 * Version History
 * Author                   Version   Date                       Description
 *========================  ======= ======== ==========================================================
 * @Author Lorenzo Pedroza  v 0.1   01/07/18      Created basic proof of concept to test a multithreaded Revtrixbot
 */

package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.DogeCV;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "Meet 3 REVTrixbot Depot", group = "Meet 3")
@Disabled //TODO uses this as a Start for Interleague tournament. Just need to get vision working
public class Meet_3_FourWheels_Depot extends OpMode {
    REVTrixbot robot = new REVTrixbot();

    volatile private boolean isLanded = false; //volatile as modified by different threads https://www.javamex.com/tutorials/synchronization_volatile.shtml
    volatile private boolean isUnhooked = false;
    volatile private boolean isInDepot = false;

    volatile private static String driveTrainStatus = "Drivetrain Status";
    volatile private static String mineralLifterStatus = "Mineral Lifter Status";
    volatile private static String goldLocatorStatus = "Gold Locator Status";
    volatile private static String teamIdentifierDepositorStatus = "Team Identifier Depositor Status";

    private static final String INITIALIZED = "INITIALIZED";
    private static final String EXECUTION_COMPLETE_STRING = "Execution Complete";

    volatile private boolean visible = false;
    volatile private Pos pos = Pos.UNKNOWN;
    volatile private double x = 0.0;
    volatile private double y = 0.0;
    volatile private double counter = 0.0; //my code vision team please help us.


    @Override
    public void init() {  //Define the behaviour of systems and initialize hardware
        //Drivetrain Code
        robot.threadDT = new REVTrixbot.REVTrixbotMTDrivetrain(){
            @Override
            public void run() {
                super.run();
                driveTrainStatus = "Waiting for Landing";
                while(!isLanded);
                driveTrainStatus = "Unhooking from lander";
                encoderDrive(1, 5.5, -5.5); //TODO work on zero radius turn method or improve turnAngleRadius drive method for pivoting on axis
                //or tunrAngleRadiusDrive(0.5, 20, 0 radius)
                isUnhooked = true;
                //turnAngleRadiusDrive(0.5, -160, 10 ); //TODO. Figure our driving to gold position. Maybe need elipsoid method?
                driveTrainStatus = "Moving to gold location judgement position";
                encoderDrive(1, 3, 3); //this also gives gold locator time to  decide

                switch (robot.goldLocator.getGoldPos()){
                    case LEFT:
                        driveTrainStatus = "Going to Left";

                       // encoderDrive(1, -6, 6);
                        //turnAngleRadiusDrive(0.5, -270, 10);
                        break;

                    case RIGHT:
                        driveTrainStatus = "Going to right";
                        //turnAngleRadiusDrive(0.5, 180, 10);
                        break;

                    case MID:
                        driveTrainStatus = "Going to middle";
                        break;

                    default:
                        driveTrainStatus = "Unknown Trajectory. Staying put";
                        break;
                }
                try {
                    sleep(5000);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
                driveTrainStatus = EXECUTION_COMPLETE_STRING;
            }
        };
        robot.threadDT.initHardware(hardwareMap);
        driveTrainStatus = INITIALIZED;

        //Mineral Lifter Code
        robot.threadMineralLifter = new REVTrixbot.REVTrixbotMTMineralLifter();
        robot.threadMineralLifter.threadedLinearActuatorArm = new REVTrixbot.REVTrixbotMTMineralLifter.ThreadedLinearActuatorArm();
        robot.threadMineralLifter.threadedArmLifter = new REVTrixbot.REVTrixbotMTMineralLifter.ThreadedArmLifter(){ //define the arm behaviour

            private void manuallyGoToAndSetZeroPositionAfterLanding(double speed){ //need to do this manually as they are not yet limit switches
                //manually move to zero position(need to put movement code)
                motor.setTargetPosition(30); //TODO set target position
                motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motor.setPower(-Math.abs(speed));
                while(motor.isBusy()); //wait for motor to reach position
                motor.setPower(0);
                //at end
                motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }

            private void unhookLiftingSupportPiece(double speed){ //need to do this manually as they are not yet limit switches
                //manually move to zero position(need to put movement code)
                motor.setTargetPosition(-500);
                motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motor.setPower(-Math.abs(speed));
                while(motor.isBusy()); //wait for motor to reach position
                motor.setPower(0);
                //at end
                motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }

            @Override
            public void run() {
                super.run();
                mineralLifterStatus = "Landing";
                unhookLiftingSupportPiece(1); //TODO decide if breaking is necesarry
                moveToRotationCount(0.5, 3500);
               //             moveRotations(1, 2500);//TO simulate falling/remove this when actually on lander.
                isLanded = true;
                while(!isUnhooked); //wait for robot to unhookLiftingSupportPiece itself
                //mineralLifterStatus = "Retracting Arm";
                //manuallyGoToAndSetZeroPositionAfterLanding(0.1);//moveToMinPos(0.1);
                /*
                mineralLifterStatus = "Moving to highest position";
                moveToHighestPosition(0.1);
                 */
                //           manuallyGoToAndSetZeroPositionAfterLanding(0.1);
                mineralLifterStatus = "Waiting 1 second";
                try { //TEMPORARY to simulate the time it takes for above statement
                    sleep(1000); //allow time for robot to fall
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
                mineralLifterStatus = EXECUTION_COMPLETE_STRING;
            }
        };
        robot.threadMineralLifter.initHardware(hardwareMap);
        mineralLifterStatus = INITIALIZED;

        //Gold Locator Code
        robot.goldLocator = new GoldMineralDetector_2(){ //redefine behaviour
            @Override
            public void tune() {
                super.tune();
                alignSize = 640; // How wide (in pixels) is the range in which the gold object will be aligned. (Represented by green bars in the preview)
                alignPosOffset = 0; // How far from center frame to offset this alignment zone.
                downscale = 0.4; // How much to downscale the input frames

                areaScoringMethod = DogeCV.AreaScoringMethod.PERFECT_AREA;
                perfectAreaScorer.perfectArea = 2400;
                perfectAreaScorer.weight = 50;

                ratioScorer.weight = 50;
                ratioScorer.perfectRatio = 1.25;
            }
        };
        robot.goldLocationUpdater = new Thread(robot.goldLocator){

            private final static int MIDPOINT = 0;
            private final static int LEFTPOINT = -106;
            private final static int RIGHTPOINT = 106;


            private static final int KNOCKOFFABLE_GOLD_MINERAL_THRESHOLD = 200; //shoulbe be greate than or equal to this value

            @Override
            public void run() {
                super.run();
                goldLocatorStatus = "Active";
                while(!isInterrupted()){
                    visible = robot.goldLocator.isFound();
                    x = robot.goldLocator.getXPosition() - MIDPOINT;
                    y = robot.goldLocator.getYPosition();

                    if(driveTrainStatus == "Moving to gold location judgement position")  //my code, vision, team we need to work on this. I have little expertise in this region. Made counter as visible tends to fluctuate, due to while loop.
                    {
                        if (visible && y > KNOCKOFFABLE_GOLD_MINERAL_THRESHOLD)
                            counter++;
                    }

                    if (robot.goldLocator.getArea() < 1200 )
                        visible = false;

                    if (robot.goldLocator.getRatio() > 2.5)
                        visible = false;

                    if (robot.goldLocator.getScore() > 10)
                        visible = false;

                    if (robot.goldLocator.getYPosition() < 200) //was 120
                        visible = false;




                    if(/*visible*/ counter > 0) {
                        if (x < 0)
                            pos = Pos.MID;
                        else if (x >= 0)
                            pos = Pos.RIGHT;
                    }   else {
                            pos = Pos.LEFT;
                    }
                    if (counter > 600)
                        break;
                    robot.goldLocator.updateGoldPos(pos);

                    /*TODO moveRotations this to the specialized telementry reporting funciotn in OPMOde
                    telemetry.addData("IsFound", visible);
                    telemetry.addData("X Pos", x);
                    telemetry.addData("Pos", pos);
                    telemetry.update();
                    */
                }
                goldLocatorStatus = "Inactive"; //don't know if this will ever display;
            }
        };
        robot.goldLocator.tune();
        robot.goldLocator.initHardware(hardwareMap);
        goldLocatorStatus = INITIALIZED;

        //teamidentifier depositor code
        robot.threadTeamIdentifierDepositor = new REVTrixbot.REVTrixbotMTTeamIdentifierDepositor(){
            @Override
            public void run() {
                super.run();
                teamIdentifierDepositorStatus = "Waiting for robot to reach depot and orient for deposit";
               // while (!isInDepot);
                depositTeamIdentifier();
                teamIdentifierDepositorStatus = "Depositing Team Identifier";
                try { //sleep to give time to show message
                    sleep(750);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
                teamIdentifierDepositorStatus = EXECUTION_COMPLETE_STRING;
            }
        };
        robot.threadTeamIdentifierDepositor.initHardware(hardwareMap);
        teamIdentifierDepositorStatus = INITIALIZED;
    }

    @Override
    public void internalPostInitLoop() {
        super.internalPostInitLoop();
        telemetry.addData("Drivetrain Status", driveTrainStatus);
        telemetry.addData("Team Identifier Depositor Status", teamIdentifierDepositorStatus);
        telemetry.addData("Mineral Lifter Status", mineralLifterStatus);
        telemetry.addData("Gold Locator Status", goldLocatorStatus);
        telemetry.addData("   Is Found", visible);
        telemetry.addData("   X Pos", x);
        telemetry.addData("   Y Pos", y);
        telemetry.addData("   Found Confidence Counter", counter);
        telemetry.addData("   Gold Pos", pos);
    }

    @Override
    public void start() {  //Start threads
        super.start();
        robot.threadDT.start();
         //TODO see if telemetry.update necesarry or use a telemetry loop

        robot.goldLocator.enable();
        robot.goldLocationUpdater.start();
        //robot.threadTeamIdentifierDepositor.start();

        robot.threadMineralLifter.threadedArmLifter.start();


    }

    @Override
    public void loop() {  //Control threads as needed.

    }

    @Override
    public void internalPostLoop() { //for updating telemetry per FTC Javadocs.
        super.internalPostLoop();
        telemetry.addData("Drivetrain Status", driveTrainStatus);
        telemetry.addData("Team Identifier Depositor Status", teamIdentifierDepositorStatus);
        telemetry.addData("Mineral Lifter Status", mineralLifterStatus);
        telemetry.addData("Gold Locator Status", goldLocatorStatus);
        telemetry.addData("   Is Found", visible);
        telemetry.addData("   X Pos", x);
        telemetry.addData("   Y Pos", y);
        telemetry.addData("   Found Confidence Counter", counter);
        telemetry.addData("   Gold Pos", pos);
    }

    @Override
    public void stop() { //Interrupt threads
        super.stop();
        robot.threadDT.interrupt();
        robot.threadMineralLifter.interrupt();
        robot.goldLocationUpdater.interrupt();//suspect the loop may also be stop issue. Maybe calle retun; in public void run() when isInterru[ted?
        //robot.goldLocator.disable(); //keep this unexecuted as I fear it may be an issue per thier documentation
        //robot.threadTeamIdentifierDepositor.interrupt();
    }

}
