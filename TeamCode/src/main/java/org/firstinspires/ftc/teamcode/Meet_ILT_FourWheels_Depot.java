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

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Meet ILT  Depot Then Left Faulty", group="REVTrixbot")
@Disabled
public class Meet_ILT_FourWheels_Depot extends LinearOpMode {


    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private GoldMineralDetector_2 locator = null;
    //private Lookeebot_4Wheels robot = null;
    private REVTrixbot robot = new REVTrixbot();

    private boolean visible = false;
    private boolean done = false;
    private double x = 0.0;
    private double y = 0.0;
    private final static int MIDPOINT = 0;  // screen midpoint
    private final static int LEFTPOINT = -106;
    private final static int RIGHTPOINT = 106;

    private volatile String mineralLifterStatus = "";
    private volatile boolean isLanded = false;
    private volatile boolean isUnhooked = false;
    private static final String EXECUTION_COMPLETE_STRING = "EXECUTION COMPLETE";


    static final double     COUNTS_PER_MOTOR_REV    = 4 ;
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;

    // REVTrix specific drive train members.
    static final double     WHEEL_DIAMETER_INCHES   = 3.5 ; //estimate
    static final double     DRIVE_WHEEL_SEPARATION  = 28.0 ; //estimate
    static final DcMotor.RunMode RUNMODE = DcMotor.RunMode.RUN_USING_ENCODER;

    public enum Pos {
        LEFT, MID, RIGHT, UNKNOWN
    }


    @Override
    public void runOpMode() {

        Pos pos = Pos.MID;
        String text = "??";

        locator = new GoldMineralDetector_2();
        locator.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        locator.useDefaults();

        // Optional Tuning
        locator.alignSize = 640; // How wide (in pixels) is the range in which the gold object will be aligned. (Represented by green bars in the preview)
        locator.alignPosOffset = 0; // How far from center frame to offset this alignment zone.
        locator.downscale = 0.4; // How much to downscale the input frames

        //locator.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        locator.areaScoringMethod = DogeCV.AreaScoringMethod.PERFECT_AREA; // Can also be PERFECT_AREA
        locator.perfectAreaScorer.perfectArea = 2400;  // Roughly cal
        //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        //locator.maxAreaScorer.weight = 0.005;
        locator.perfectAreaScorer.weight = 0.01;

        locator.ratioScorer.weight = 50;
        locator.ratioScorer.perfectRatio = 1.25;  // To be calibrated
        locator.enable();

        telemetry.addData("locator", "Initialized");
        telemetry.update();

        // Init robot


        robot.dt.initHardware(hardwareMap);
        robot.idenfierFor5197Depositer.initHardware(hardwareMap);

        robot.threadMineralLifter = new REVTrixbot.REVTrixbotMTMineralLifter(){
            @Override
            public void initHardware(HardwareMap ahwMap) { //do not init servos.
                threadedArmLifter.initHardware(ahwMap);
                threadedLinearActuatorArm.initHardware(ahwMap);
            }
        };
        robot.threadMineralLifter.threadedLinearActuatorArm = new REVTrixbot.REVTrixbotMTMineralLifter.ThreadedLinearActuatorArm();
        robot.threadMineralLifter.threadedArmLifter = new REVTrixbot.REVTrixbotMTMineralLifter.ThreadedArmLifter(){
            private void manuallyGoToAndSetZeroPositionAfterLanding(double speed){ //need to do this manually as they are not yet limit switches
                //manually move to zero position(need to put movement code)
                motor.setTargetPosition(600); //TODO set target position
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
                try {
                    sleep(1500); //let the other bots go first
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
                unhookLiftingSupportPiece(1); //TODO decide if breaking is necesarry
                moveToRotationCount(0.5, 3500);
                //             moveRotations(1, 2500);//TO simulate falling/remove this when actually on lander.
                isLanded = true;
                while(!isUnhooked); //wait for robot to unhookLiftingSupportPiece itself
                mineralLifterStatus = "Retracting Arm";
                manuallyGoToAndSetZeroPositionAfterLanding(0.7);//moveToMinPos(0.1);
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
        //robot.revTrixBotMineralArm.laArmLifter.teleOpMoveWithButtons(false, true, 0.008); //sleezy but will have to do

        //robot.roverRuckusRevTrixBotLift.initHardware(hardwareMap);

        // turn on camera
        locator.enable();

        done = false;

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();



        robot.threadMineralLifter.start();

        sleep(5500); //wait for landing

        robot.dt.encoderDrive(1, 5.05, -5.05); //TODO work on zero radius turn method or improve turnAngleRadius drive method for pivoting on axis
        //or tunrAngleRadiusDrive(0.5, 20, 0 radius)
        isUnhooked = true;



        robot.dt.encoderDrive(1, 6, 6);
        //sleep(1000);
        sleep(1400);


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive() && !done) {
        // lineup the camera on the right side
        // right 2 balls are visible
            //sssland(); //starts the robot in the middle, then turn to the right before sampling

            visible = locator.isFound();
            x = locator.getXPosition() - MIDPOINT;
            y = locator.getYPosition();


            if (locator.getArea() < 1200 )
                visible = false;


            if (locator.getRatio() > 2.5)
                visible = false;

            if (locator.getScore() > 10)
                visible = false;


            if (locator.getYPosition() < 120)
                visible = false;


            if(visible) {
                if (x < 0)
                    pos = Pos.MID;
                else if (x >= 0)
                    pos = Pos.RIGHT;
            }   else {
                pos = Pos.LEFT;
            }


            switch (pos) {
                case LEFT:
                    //do left thing
                    text = "LEFT";
                    targetLeft();

                    break;

                case RIGHT:
                    //do left thing
                    text = "RIGHT";
                    targetRight();

                    break;

                case MID:
                    //do left thing
                    text = "Mid";
                    targetCenter();
                    break;

                default:
                    text = "Unknown";
                    targetUnknown();
                    break;

            }

            telemetry.addData("IsFound" ,visible); // Is the bot aligned with the gold mineral
            telemetry.addData("X Pos" , x); // Gold X pos.
            telemetry.addData("Pos" , text); // Gold X pos.

            telemetry.update();// Gold X pos.

            /*
            robot.roverRuckusRevTrixBotLift.moveToMinPos(1);
            */
        }

        telemetry.addData("Status" ,"All Done"); // Is the bot aligned with the gold mineral
        telemetry.update();// Gold X pos.
        robot.threadMineralLifter.interrupt();
    }


    private void land(){
       // robot.revTrixBotMineralArm.laArmLifter.setBraking(false);
        robot.revTrixBotMineralArm.laArmLifter.moveToMaxPos(0.1);
        /*
        robot.dt.encoderDrive(1, 3.3, -3.3); //turn to gold
        */
        sleep(2000);
    }



    private void targetLeft()  {
        // build a profile to handle target on left
        robot.dt.encoderDrive(1, -18, 18);
        //sleep(1000);
        robot.dt.encoderDrive(1, 17, 17);
        //sleep(1000);
        robot.dt.encoderDrive(1, 16, -16);
        //sleep(1000)
        robot.dt.encoderDrive(1, 28.3, 28.3);
        //sleep(1000);
        robot.dt.encoderDrive(1, 48.9, -48.9);
        //sleep(1000);
        robot.dt.encoderDrive(1,-6,-6);
        //sleep(1000);
        robot.idenfierFor5197Depositer.depositTeamIdentifier();
        //sleep(1000);
        robot.dt.encoderDrive(1, 70, 70);
        //sleep(3000);
        //robot.dt.encoderDrive(1, 8, 8);
        //sleep(1000);
        //robot.dt.encoderDrive(1, 5, -5);
        //sleep(1000);
        //robot.dt.encoderDrive(1, 40, 40);
        //sleep(2000);
        done = true;


    }

    private void targetRight() {
        // build a profile to handle target on right
        robot.dt.encoderDrive(1, 3, -3);
        //sleep(1000);// wait for the previous motion to complete
        robot.dt.encoderDrive(1, 37, 37);
        //sleep(1500);
        robot.dt.encoderDrive(1, 23, -23);
        //sleep(1000);
        robot.dt.encoderDrive(1, -20, -20);
        //sleep(1000);
        robot.idenfierFor5197Depositer.depositTeamIdentifier();
        //sleep(1000);
        robot.dt.encoderDrive(1, -1, 1);
        //sleep(1000);
        robot.dt.encoderDrive(1, 75, 75);
        //sleep(2000);
        done = true;  // end the run
    }

    private void targetCenter() {

        // build a profile to handle target on right
        robot.dt.encoderDrive(1, -5, 5); //turn to gold
        //sleep(1000);
        robot.dt.encoderDrive(1, 30,30); //straight
        //sleep(1000);
        robot.dt.encoderDrive(1, -10, 10); //small left turn
        //sleep(1000);
        robot.dt.encoderDrive(1, 12.7, 12.7); //go straight again was 14
        //sleep(1000);
        robot.dt.encoderDrive(1, 20, -20); //turn to corner
        //sleep(1000);
        robot.dt.encoderDrive(1, 14.9, 14.9); //forward to corner
        //sleep(1000);
        robot.dt.encoderDrive(1, 18.85, -18.85);
        //sleep(1000);
        robot.dt.encoderDrive(1, 4, 4);
        //sleep(1000);
        robot.idenfierFor5197Depositer.depositTeamIdentifier();
        //sleep(1000);

        robot.dt.encoderDrive(1, 78, 78);
        //sleep(3000);
        done = true;
    }



    private void targetUnknown() {
        // Should not get here
        done = true;  // end the run
    }

}
