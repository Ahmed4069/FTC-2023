package org.firstinspires.ftc.teamcode.Autonomous.Auto;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
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
public class AutoLeft extends LinearOpMode {
    Robot robot;
    Scheduler scheduler;
    Scheduler ParkingScheduler1;
    Scheduler ParkingScheduler2;
    Scheduler ParkingScheduler3;
    SignalClassifier classifier;
    boolean done  = false;

    @Override
    public void runOpMode() {
        robot = new Robot(hardwareMap, telemetry);
        classifier = new SignalClassifier(hardwareMap, telemetry);

        scheduler = new Scheduler(robot, telemetry);
        scheduler.addCommand(new DriveToPosCommand(0.5, 86000));
        scheduler.addCommand(new TurnToAngle(0.5, Math.toRadians(-51)));
        scheduler.addCommand(new DriveToPosCommand(0.5, 8100));
        scheduler.addCommand(new RaiseArm(4, 0, 0));
        scheduler.addCommand(new waitCommand(2));
        scheduler.addCommand(new RaiseArm(3, 0, 0));
        scheduler.addCommand(new waitCommand(10));
        scheduler.addCommand(new IntakeOn(IntakeOn.Use.OUT));
        scheduler.addCommand(new waitCommand(1));
        scheduler.addCommand(new IntakeOff());
        // scheduler.addCommand(new DriveToPosCommand(-0.5, -5000));
        scheduler.addCommand(new RaiseArm(0, 0, 0));
        scheduler.addCommand(new TurnToAngle(0.5, -82));
        scheduler.addCommand(new DriveToPosCommand(-0.5, -45000));
        scheduler.addCommand(new IntakeOn(IntakeOn.Use.IN));
        scheduler.addCommand(new waitCommand(1));
        scheduler.addCommand(new IntakeOff());
        scheduler.addCommand(new DriveToPosCommand(0.5, 45000));
        scheduler.addCommand(new RaiseArm(4, 0, 0));
        scheduler.addCommand(new IntakeOn(IntakeOn.Use.OUT));
        scheduler.addCommand(new waitCommand(1));
        scheduler.addCommand(new RaiseArm(0, 0, 0));
        //scheduler.addCommand(new TurnToAngle(0.5, Math.toRadians(85)));

        ParkingScheduler1 = new Scheduler(robot, telemetry);
        ParkingScheduler1.addCommand(new DriveToPosCommand(-0.5, -6000));
        ParkingScheduler1.addCommand(new TurnToAngle(0.5, Math.toRadians(-90)));
        ParkingScheduler1.addCommand(new DriveToPosCommand(-0.5, -40000));
        ParkingScheduler1.addCommand(new TurnToAngle(0.5, 0));

        ParkingScheduler2 = new Scheduler(robot, telemetry);
        //ParkingScheduler2.addCommand(new TurnToAngle(0.5, Math.toRadians(10)));
        ParkingScheduler2.addCommand(new DriveToPosCommand(-0.5, -6000));
        ParkingScheduler2.addCommand(new TurnToAngle(0.5, Math.toRadians(0)));

        ParkingScheduler3 = new Scheduler(robot, telemetry);
        ParkingScheduler3.addCommand(new DriveToPosCommand(-0.5, -6000));
        //ParkingScheduler3.addCommand(new TurnToAngle(0.5, Math.toRadians(45)));
        ParkingScheduler3.addCommand(new TurnToAngle(0.5, Math.toRadians(90)));
        ParkingScheduler3.addCommand(new DriveToPosCommand(-0.5, -40000));
        ParkingScheduler3.addCommand(new TurnToAngle(0.5, 0));


        waitForStart();
        //sleep(5000);
        int signal = classifier.classify();

        telemetry.addLine(String.valueOf(signal));

        telemetry.addData("NumOfCommands", scheduler.getListSize());
        telemetry.update();
        while (opModeIsActive() && scheduler.getListSize() != 0) {
            scheduler.run();
            telemetry.update();
            idle();
        }
        while (opModeIsActive() && !done){
            telemetry.addData("signal is ", signal);
            if(signal == 1){
                while (opModeIsActive() && ParkingScheduler1.getListSize() != 0){
                    ParkingScheduler1.run();
                    telemetry.update();
                    idle();
                }
                done = true;
            }
            else if(signal == 3){
                while (opModeIsActive() && ParkingScheduler3.getListSize() != 0){
                    ParkingScheduler3.run();
                    telemetry.update();
                    idle();
                }
                done = true;
            }
            else {
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

