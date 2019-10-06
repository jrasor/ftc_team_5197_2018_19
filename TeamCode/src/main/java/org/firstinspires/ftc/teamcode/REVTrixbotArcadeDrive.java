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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * This file provides basic Teleop driving for a REVTrixbot. It is modified and simplified
 * from the FtcRobotController external sample PushbotTeleopTankIterative. The code is structured
 * as an Iterative OpMode. The arm and claw operation in the sample is not implemented here.
 *
 * Tank drive means the gamepad left stick controls the left motors, and the right stick controls
 * the right motors.
 *
 * This OpMode uses the REVTrixbot hardware class to define the devices on the robot.
 * All device access is managed through the REVTrix class.
 *
 * Version history
 * ======= ======
 * v 0.1    10/11/18 jmr primitive version, just enough to test the drive train. Does not extend
 *          GenericRobot class.
 *
 */

@TeleOp(name="REVTrixbot: Teleop Arcade", group="REVTrixbot")
@Disabled
public class REVTrixbotArcadeDrive extends ModularRobotIterativeOpMode {

    /* Declare OpMode members. */
    private REVTrixbot robot       = new REVTrixbot();  // Class created to define a REVTrixbot's hardware


    /*
     * Code to run ONCE when the driver hits INIT
     */

    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The initHardware() method of the hardware class does all the work here
         */
        robot.dt.initHardware(hardwareMap);

        robot.revTrixbotMineralPaddles.initHardware(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello, Driver!");

    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {


        robot.dt.teleOpArcadeDrive(gamepad1, F310JoystickInputNames.Joysticks.LEFT_STICK);
        robot.revTrixbotMineralPaddles.teleOpDeployRetractPaddles(gamepad1.y);

        telemetry.addData("left",  "%.2f", -gamepad1.left_stick_y); //TODO make method for Arcade drive for this in drivetrain classes.
        telemetry.addData("right", "%.2f", -gamepad1.right_stick_y);
        telemetry.addData("Rotations", robot.dt.getCurrentAverageDTPosition(true));


        telemetry.addData("IsFound" ,robot.goldLocator.isFound()); // Is the bot aligned with the gold mineral
        telemetry.addData("X Pos" , robot.goldLocator.getXPosition()); // Gold X pos.
        telemetry.addData("Pos" , robot.goldLocator.getGoldPosForTelemetry()); // Gold X pos.

        //telemetry.update();// Gold X pos.

        //telemetry.addData("Status" ,"All Done"); // Is the bot aligned with the gold mineral
        //telemetry.update();// Gold X pos.
    }


    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}
