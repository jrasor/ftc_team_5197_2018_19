/**Limited Motion Actuator Tester Program
 *      This program is intended for use on limited motion actuator.
 *      WARNING. Failure to pay attention will result in damage to the robot.
 *
 * Version History
 * =====================
 * v 0.1 @author Lorenzo Pedroza
 * v 0.4 @author Lorenzo Pedroza
 * v 0.5 @author Lorenzo Pedroza Now can select custom motor names using the "EHXmotorX" naming convention
 *
 * */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.Random;

@TeleOp(name = "LM Actuator Tester Program",group = "Tester Programs")
@Disabled
public class LimitedMotionActuatorTesterProgram extends LinearOpMode {
    Random random =  new Random();
    F310JoystickInputNames.ABXYButtons[] randomButton =
            new F310JoystickInputNames.ABXYButtons[]{F310JoystickInputNames.ABXYButtons.A_BUTTON,
            F310JoystickInputNames.ABXYButtons.B_BUTTON, F310JoystickInputNames.ABXYButtons.X_BUTTON,
            F310JoystickInputNames.ABXYButtons.Y_BUTTON};
    F310JoystickInputNames.ABXYButtons chosenButton = randomButton[random.nextInt(randomButton.length)];
    String disclaimer = "DISCLAIMER";
    String warningMessage = "LIMITED MOTION ACTUATOR TESTER PROGRAM v 0.5\n" +
            " to continue. Software is NOT LIABLE for damage done to robot\n as a result of not being careful.\n" +
            "Connect motor named \"EH1motor0\" in configuration before continuing\n";
    String motorName = "";


    @Override
    protected void handleLoop() {
        super.handleLoop(); //get it ot stop nagging when hit stop button.
        if(isStopRequested())
            requestOpModeStop();
    }

    @Override
    public void runOpMode() {
        //sleep(5000);


        telemetry.addData("Motor Name", "Input Motor Name(You can hit play by the way(though nothing will happen))");
        telemetry.addData("Expansion Hub Name", "A for EH1, B for EH2");
        telemetry.update();
        while (!gamepad1.a || !gamepad1.b){//bad practice, but see below statements to justify it
            if(gamepad1.a)
            {
                motorName += "EH1";
                break;
            }
            else if (gamepad1.b)
            {
                motorName += "EH2";
                break;
            }
            handleLoop();
        }
        telemetry.clearAll();
        telemetry.update();

        telemetry.addData("Motor Port Name", "A for motor0, B for motor1, X for motor2, Y for motor3");
        telemetry.update();
        sleep(1000);
        while (!gamepad1.a || !gamepad1.b || !gamepad1.x || !gamepad1.y){
            if(gamepad1.a)
            {
                motorName += "motor0";
                break;
            }
            else if (gamepad1.b)
            {
                motorName += "motor1";
                break;
            }
            else if (gamepad1.x)
            {
                motorName += "motor2";
                break;
            }
            else if (gamepad1.y)
            {
                motorName+= "motor3";
                break;
            }
            handleLoop();
        }

        TestableLimitedMotionActuator testMotor = new TestableLimitedMotionActuator(motorName);

        telemetry.clearAll();
        telemetry.update();

        switch (chosenButton) {
            case A_BUTTON:
                telemetry.addData(disclaimer, warningMessage + "Press: a to continue");
                telemetry.update();
                while (!gamepad1.a){handleLoop();}
                testMotor.initHardware(hardwareMap);
                    break;
            case B_BUTTON:
                telemetry.addData(disclaimer, warningMessage + "Press: b to continue");
                telemetry.update();
                while (!gamepad1.b){handleLoop();}
                testMotor.initHardware(hardwareMap);
                    break;
            case X_BUTTON:
                telemetry.addData(disclaimer, warningMessage + "Press: x to continue");
                telemetry.update();
                while (!gamepad1.x){handleLoop();}
                testMotor.initHardware(hardwareMap);
                    break;
            case Y_BUTTON:
                telemetry.addData(disclaimer, warningMessage + "Press: y to continue");
                telemetry.update();
                while (!gamepad1.y){handleLoop();}
                testMotor.initHardware(hardwareMap);
                    break;
            default:
                handleLoop();
        }



        //telemetry.clearAll();
        waitForStart();
        telemetry.addData("Test", "OP MODE ACTIVE");
        telemetry.update();

        while(opModeIsActive())
        {
            telemetry.addData("Roatations", testMotor.getCurrentPosition());
            telemetry.addData("maxLimit", testMotor.maxLimit);
            telemetry.addData("Speed", testMotor.speed);
            telemetry.addData("Motor Direction", testMotor.motor.getDirection());
            telemetry.addData("Controls", "A: Add rotaionst, B: -rotations," +
                    " D-pad up: +rotationLimit, D-pad down: -rotation limit, Both Bumpers: reverse motor direction");
            telemetry.update();

            testMotor.teleOpSetSpeed(gamepad1);
            testMotor.teleOpMoveWithButtons(gamepad1.a, gamepad1.b, testMotor.speed);
            testMotor.teleOpIncrementUpperLimit(gamepad1.dpad_up);
            testMotor.teleOpDecrementUpperLimit(gamepad1.dpad_down);
            testMotor.teleOpSwitchDirection(gamepad1.left_bumper && gamepad1.right_bumper);

        }
    }
}

