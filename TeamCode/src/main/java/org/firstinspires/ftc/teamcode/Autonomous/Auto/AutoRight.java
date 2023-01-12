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
public class AutoRight extends LinearOpMode{
    Robot robot;
    Scheduler scheduler;
    SignalClassifier classifier;

    @Override
    public void runOpMode() {
        robot = new Robot(hardwareMap, telemetry);
        scheduler = new Scheduler(robot, telemetry);
        classifier = new SignalClassifier(hardwareMap, telemetry);

        scheduler.addCommand(new DriveToPosCommand(0.5, 85000));
        scheduler.addCommand(new TurnToAngle(0.5, Math.toRadians(51)));
        scheduler.addCommand(new DriveToPosCommand(0.5, 5000));
        scheduler.addCommand(new RaiseArm(4, 0, RaiseArm.Options.Junction));
        scheduler.addCommand(new IntakeOn(IntakeOn.Use.OUT));
        scheduler.addCommand(new waitCommand(10));
        scheduler.addCommand(new IntakeOff());
        // scheduler.addCommand(new DriveToPosCommand(-0.5, -5000));
        scheduler.addCommand(new RaiseArm(0, 0, RaiseArm.Options.Junction));
        scheduler.addCommand(new TurnToAngle(0.5, Math.toRadians(85)));

        if(classifier.classify() == "ConeSignal1"){
            scheduler.addCommand(new DriveToPosCommand(-0.5, -1000));
            scheduler.addCommand(new TurnToAngle(0.5, Math.toRadians(90)));
            scheduler.addCommand(new DriveToPosCommand(0.5, 85000 / 2));
            scheduler.addCommand(new TurnToAngle(0.5, Math.toRadians(0)));
        }
        else if (classifier.classify() == "ConeSignal3"){
            scheduler.addCommand(new DriveToPosCommand(-0.5, -1000));
            scheduler.addCommand(new TurnToAngle(0.5, Math.toRadians(-90)));
            scheduler.addCommand(new DriveToPosCommand(0.5, 85000 / 2));
            scheduler.addCommand(new TurnToAngle(0.5, Math.toRadians(0)));
        }
        else{
            scheduler.addCommand(new DriveToPosCommand(-0.5, -1000));
            scheduler.addCommand(new TurnToAngle(0.5, Math.toRadians(0)));
        }

        waitForStart();

        while(opModeIsActive() && scheduler.getListSize() != 0) {
            scheduler.run();
            telemetry.update();
            idle();
        }

        telemetry.addLine("done");
        telemetry.update();

        while (opModeIsActive()) {
        }

    }
}
