package org.firstinspires.ftc.teamcode.Autonomous.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Autonomous.Commands.DriveToPosCommand;
import org.firstinspires.ftc.teamcode.Autonomous.Commands.TurnToAngle;
import org.firstinspires.ftc.teamcode.Autonomous.Scheduler;
import org.firstinspires.ftc.teamcode.Main.Robot.Robot;
import org.firstinspires.ftc.teamcode.Vision.SignalClassifier;

@Autonomous
public class Park extends LinearOpMode {
    Scheduler ParkingScheduler1, ParkingScheduler2, ParkingScheduler3;
    Robot robot;
    SignalClassifier classifier;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap, telemetry);
        classifier = new SignalClassifier(hardwareMap, telemetry);


        ParkingScheduler1 = new Scheduler(robot, telemetry);
        ParkingScheduler1.addCommand(new DriveToPosCommand(-0.5, -6000));
        ParkingScheduler1.addCommand(new TurnToAngle(0.5, Math.toRadians(90)));
        ParkingScheduler1.addCommand(new DriveToPosCommand(-0.5, -50000));
        ParkingScheduler1.addCommand(new TurnToAngle(0.5, 0));

        ParkingScheduler2 = new Scheduler(robot, telemetry);
        //ParkingScheduler2.addCommand(new TurnToAngle(0.5, Math.toRadians(10)));
        ParkingScheduler2.addCommand(new DriveToPosCommand(-0.5, -6000));
        ParkingScheduler2.addCommand(new TurnToAngle(0.5, Math.toRadians(0)));

        ParkingScheduler3 = new Scheduler(robot, telemetry);
        ParkingScheduler3.addCommand(new DriveToPosCommand(-0.5, -6000));
        //ParkingScheduler3.addCommand(new TurnToAngle(0.5, Math.toRadians(45)));
        ParkingScheduler3.addCommand(new TurnToAngle(0.5, -Math.toRadians(90)));
        ParkingScheduler3.addCommand(new DriveToPosCommand(-0.5, -50000));
        ParkingScheduler3.addCommand(new TurnToAngle(0.5, 0));


    }
}
