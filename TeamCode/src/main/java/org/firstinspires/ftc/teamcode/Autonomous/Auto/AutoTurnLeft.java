package org.firstinspires.ftc.teamcode.Autonomous.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Autonomous.Commands.DriveToPosCommand;
import org.firstinspires.ftc.teamcode.Autonomous.Commands.IntakeOff;
import org.firstinspires.ftc.teamcode.Autonomous.Commands.IntakeOn;
import org.firstinspires.ftc.teamcode.Autonomous.Commands.RaiseArm;
import org.firstinspires.ftc.teamcode.Autonomous.Commands.TurnToAngle;
import org.firstinspires.ftc.teamcode.Autonomous.Commands.waitCommand;
import org.firstinspires.ftc.teamcode.Autonomous.Scheduler;
import org.firstinspires.ftc.teamcode.Main.Robot.Robot;
import org.firstinspires.ftc.teamcode.Vision.SignalClassifier;

@Autonomous
public class AutoTurnLeft extends LinearOpMode {
    Robot robot;
    Scheduler scheduler;

    @Override
    public void runOpMode() {
        robot = new Robot(hardwareMap, telemetry);

        scheduler = new Scheduler(robot, telemetry);
        scheduler.addCommand(new TurnToAngle(0, Math.toRadians(-90)));

        waitForStart();

        while (opModeIsActive() && scheduler.getListSize() != 0) {
            scheduler.run();
            sleep(500);
            idle();
        }

        telemetry.addData("heading is ", robot.gyro.getHeading());
        telemetry.update();

        while (opModeIsActive()) {}

    }
}

