package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Autonomous.Commands.DriveToPosCommand;
import org.firstinspires.ftc.teamcode.Autonomous.Commands.TurnToAngle;
import org.firstinspires.ftc.teamcode.Main.Robot.Robot;
import org.firstinspires.ftc.teamcode.Vision.SignalClassifier;

@Autonomous
public class CommandAuto extends LinearOpMode{
    Robot robot;
    Scheduler scheduler;
    SignalClassifier classifier;



    @Override
    public void runOpMode() {
        robot = new Robot(hardwareMap, telemetry);
        scheduler = new Scheduler(robot, telemetry);
        classifier = new SignalClassifier(hardwareMap, telemetry);

        scheduler.addCommand(new DriveToPosCommand(0.5, 3000));
        scheduler.addCommand(new TurnToAngle(0.5, 30));
        //lift Arm and intake
        scheduler.addCommand(new TurnToAngle(0.5, -120));
        scheduler.addCommand(new DriveToPosCommand(0.5, 1500));
        scheduler.addCommand(new TurnToAngle(0.5, 90));
        scheduler.addCommand(new DriveToPosCommand(0.5, 1000));
        scheduler.addCommand(new TurnToAngle(0.5, 90));
        scheduler.addCommand(new DriveToPosCommand(-0.5, 1000));
        //get cone from cone stack
        scheduler.addCommand(new DriveToPosCommand(0.5, 1500));
        scheduler.addCommand(new TurnToAngle(0.5, -30));
        scheduler.addCommand(new DriveToPosCommand(0.2, 100));
        //lift and deposit cone


        waitForStart();
        String signal = classifier.classify();
        if(signal == "ConeSignal1"){
            scheduler.addCommand(new DriveToPosCommand(-0.5, 500));
            scheduler.addCommand(new TurnToAngle(0.5, 30));
            scheduler.addCommand(new DriveToPosCommand(-0.5, 1000));
            scheduler.addCommand(new TurnToAngle(0.5, -90));
            scheduler.addCommand(new DriveToPosCommand(0.5, 500));
        }
        else if (signal == "ConeSignal2"){
            scheduler.addCommand(new DriveToPosCommand(-0.5, 500));
            scheduler.addCommand(new TurnToAngle(0.5, 30));
            scheduler.addCommand(new DriveToPosCommand(-0.5, 1000));
        }
        else if (signal == "ConeSignal3"){
            scheduler.addCommand(new DriveToPosCommand(-0.5, 500));
            scheduler.addCommand(new TurnToAngle(0.5, 30));
            scheduler.addCommand(new DriveToPosCommand(-0.5, 1000));
            scheduler.addCommand(new TurnToAngle(0.5, 90));
            scheduler.addCommand(new DriveToPosCommand(0.5, 500));
        }
        sleep(100);

        while(opModeIsActive() && scheduler.getListSize() != 0) {
            scheduler.run();
            telemetry.update();
            idle();
        }

    }
}
