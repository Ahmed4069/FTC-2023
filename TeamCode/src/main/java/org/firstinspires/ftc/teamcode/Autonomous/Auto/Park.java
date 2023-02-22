package org.firstinspires.ftc.teamcode.Autonomous.Auto;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Autonomous.Commands.DriveToPosCommand;
import org.firstinspires.ftc.teamcode.Autonomous.Commands.TurnToAngle;
import org.firstinspires.ftc.teamcode.Autonomous.Scheduler;
import org.firstinspires.ftc.teamcode.Main.Robot.Robot;
import org.firstinspires.ftc.teamcode.Vision.SignalClassifier;

@Autonomous
public class Park extends LinearOpMode {
    Scheduler firstScheduler, ParkingScheduler1, ParkingScheduler2, ParkingScheduler3;
    Robot robot;
    SignalClassifier classifier;
    boolean done = false;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap, telemetry);
        classifier = new SignalClassifier(hardwareMap, telemetry);

        firstScheduler = new Scheduler(robot, telemetry);
        firstScheduler.addCommand(new DriveToPosCommand(0.5, (86000 - 5000) / 2));

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

        waitForStart();
        int signal = classifier.classify();

        while (opModeIsActive() && !done) {
            telemetry.addData("signal is ", signal);
            while (firstScheduler.getListSize() != 0 && opModeIsActive()){
                firstScheduler.run();
                telemetry.update();
                idle();
            }

            if (signal == 1) {
                while (opModeIsActive() && ParkingScheduler1.getListSize() != 0) {
                    ParkingScheduler1.run();
                    telemetry.update();
                    idle();
                }
                done = true;
            } else if (signal == 3) {
                while (opModeIsActive() && ParkingScheduler3.getListSize() != 0) {
                    ParkingScheduler3.run();
                    telemetry.update();
                    idle();
                }
                done = true;
            } else {
                while (opModeIsActive() && ParkingScheduler2.getListSize() != 0) {
                    ParkingScheduler2.run();
                    telemetry.update();
                    idle();
                }
                done = true;
            }
        }
    }
}
