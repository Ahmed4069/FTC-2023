package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Autonomous.Commands.DriveToPosCommand;
import org.firstinspires.ftc.teamcode.Autonomous.Commands.TurnToAngle;
import org.firstinspires.ftc.teamcode.Main.Robot.Robot;
import org.firstinspires.ftc.teamcode.Vision.SignalClassifier;

@Autonomous
public class AutoForward extends LinearOpMode{
    Robot robot;
    Scheduler scheduler;


    @Override
    public void runOpMode() {
        robot = new Robot(hardwareMap, telemetry);
        scheduler = new Scheduler(robot, telemetry);

        scheduler.addCommand(new DriveToPosCommand(0.5, 40000));
        scheduler.addCommand(new TurnToAngle(0.5, -Math.PI / 4));

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
