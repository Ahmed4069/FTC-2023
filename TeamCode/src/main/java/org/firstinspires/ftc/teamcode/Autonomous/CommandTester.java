package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Autonomous.Commands.DriveToPosCommand;
import org.firstinspires.ftc.teamcode.Autonomous.Commands.IntakeOff;
import org.firstinspires.ftc.teamcode.Autonomous.Commands.IntakeOn;
import org.firstinspires.ftc.teamcode.Autonomous.Commands.TurnToAngle;
import org.firstinspires.ftc.teamcode.Main.Robot.Robot;

@Autonomous
public class CommandTester extends LinearOpMode{
    Robot robot;
    Scheduler scheduler;


    @Override
    public void runOpMode() {
        robot = new Robot(hardwareMap, telemetry);
        scheduler = new Scheduler(robot, telemetry);

        scheduler.addCommand(new DriveToPosCommand(0.5, 40000));
        scheduler.addCommand(new TurnToAngle(0.5, -Math.PI / 4));
        scheduler.addCommand(new IntakeOn(IntakeOn.Use.IN));
        scheduler.addCommand(new IntakeOn(IntakeOn.Use.OUT));
        scheduler.addCommand(new IntakeOff());

        waitForStart();

        while(opModeIsActive() && scheduler.getListSize() != 0) {
            scheduler.run();
            telemetry.update();
            idle();
        }

        robot.driveBase.print();
        telemetry.update();

        while (opModeIsActive()) {

        }

    }
}

