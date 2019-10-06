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
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single
 * robot, a Lookeebot_4Wheels. This our Rover Ruckus competition bot. It is
 * adapted from our Trainerbot and Lookeebot.
 *
 * See the supplied external samples for FtcRobotController for usage examples
 * easily converted to run on this bot.
 *
 * Version history
 * ======= =======
 *
 * v 0.1    Bob, 10/26. First conversion from Trainerbot and Lookeebot.
 * v 0.2    jmr, 10/27. Improved documentation and fixed motor numbering errors.
 * v 0.21   jmr, 10/27. Adjusted motor specific constants to fit REV Core HEX
 *          motors. More work on documentation.
 *
 */

public class Lookeebot_4Wheels
{
    // General conversion constants.
    static final double MM_PER_IN = 25.4;   // ** to do: moveRotations to Generic robot class
    static final double IN_PER_MM = 1/MM_PER_IN;

    // Specific measurements for this robot class.
    // Motors are REV Core Hex motors, REV-41-1300.
    static final double     COUNTS_PER_MOTOR_REV    = 288 ; // 1120 or 1440 for other popular motors
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ; // No reduction.

    // Specific drive train members.
    // Wheels are REV Robotics 90mm Traction wheels, REV-41-1354.
    static final double     WHEEL_DIAMETER_MM   = 90.0 ;
    static final double     DRIVE_WHEEL_SEPARATION  = 15.2 ;    // Dual 90s on the back
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_MM * IN_PER_MM * Math.PI);

    // Specific motor and actuator members.
    public DcMotor leftFrontDrive   = null;
    public DcMotor rightFrontDrive  = null;
    public DcMotor leftBackDrive   = null;
    public DcMotor rightBackDrive  = null;
    // RobotLift and claw members go here.

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public Lookeebot_4Wheels(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize REV Core Hex motors.
        rightBackDrive  = hwMap.get(DcMotor.class, "motor0");
        leftBackDrive = hwMap.get(DcMotor.class, "motor1");
        rightFrontDrive  = hwMap.get(DcMotor.class, "motor2");
        leftFrontDrive = hwMap.get(DcMotor.class, "motor3");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);

        // Set all motors to zero power.
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);

        // Set all motors to run with encoders.
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}
