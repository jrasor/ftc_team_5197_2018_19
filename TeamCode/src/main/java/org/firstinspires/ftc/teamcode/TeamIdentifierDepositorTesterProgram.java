package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name="Team Identifier Depositor Test Program", group = "REVTrixbot System Test Programs")
public class TeamIdentifierDepositorTesterProgram extends OpMode {

    REVTrixbot robot = new REVTrixbot();
    private String teamIdentifierDepositorStatus = "UNKNOWN";

    @Override
    public void init() {
        robot.threadTeamIdentifierDepositor = new REVTrixbot.REVTrixbotMTTeamIdentifierDepositor(){
            @Override
            public void run() {
                super.run();
                teamIdentifierDepositorStatus = "Depositing team identifier";
                depositTeamIdentifier();
                teamIdentifierDepositorStatus = "Done";
            }
        };
        robot.threadTeamIdentifierDepositor.initHardware(hardwareMap);
        teamIdentifierDepositorStatus = "Initialized";
    }

    @Override
    public void internalPostInitLoop() {
        super.internalPostInitLoop();
        telemetry.addData("Team Identifier Depositor Status", teamIdentifierDepositorStatus);
    }

    @Override
    public void start() {
        super.start();
        robot.threadTeamIdentifierDepositor.start();
    }

    @Override
    public void loop() {

    }

    @Override
    public void internalPostLoop() {
        super.internalPostLoop();
        telemetry.addData("Team Identifier Depositor Status", teamIdentifierDepositorStatus);
    }

    @Override
    public void stop() {
        super.stop();
        robot.threadTeamIdentifierDepositor.interrupt();
    }

}
