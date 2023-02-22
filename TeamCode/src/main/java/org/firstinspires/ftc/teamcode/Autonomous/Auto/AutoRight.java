package org.firstinspires.ftc.teamcode.Autonomous.Auto;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Autonomous.Commands.DriveToPosCommand;
import org.firstinspires.ftc.teamcode.Autonomous.Commands.IntakeOff;
import org.firstinspires.ftc.teamcode.Autonomous.Commands.IntakeOn;
import org.firstinspires.ftc.teamcode.Autonomous.Commands.ParallelCommand;
import org.firstinspires.ftc.teamcode.Autonomous.Commands.RaiseArm;
import org.firstinspires.ftc.teamcode.Autonomous.Commands.TurnToAngle;
import org.firstinspires.ftc.teamcode.Autonomous.Commands.waitCommand;
import org.firstinspires.ftc.teamcode.Autonomous.Scheduler;
import org.firstinspires.ftc.teamcode.Main.Constants;
import org.firstinspires.ftc.teamcode.Main.Robot.Robot;
import org.firstinspires.ftc.teamcode.Vision.SignalClassifier;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.PrintWriter;

@Autonomous
public class AutoRight extends LinearOpMode {
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
        scheduler.addCommand(new DriveToPosCommand(0.7, 86000));
        scheduler.addCommand(new TurnToAngle(0.5, Math.toRadians(57)));
        scheduler.addCommand(new DriveToPosCommand(0.7, 8000));
        scheduler.addCommand(new RaiseArm(4, 0, 2500));
        scheduler.addCommand(new IntakeOn(IntakeOn.Use.OUT));
        scheduler.addCommand(new IntakeOff());
        scheduler.addCommand(new DriveToPosCommand(-0.6, -8000));
        scheduler.addCommand(new ParallelCommand(new TurnToAngle(0.5, Math.toRadians(95)), new RaiseArm(2,0, 2500)));
        scheduler.addCommand(new DriveToPosCommand(-0.7, -38000));
        scheduler.addCommand(new waitCommand(300));
        //scheduler.addCommand(new RaiseArm(2, 0));
        scheduler.addCommand(new ParallelCommand(new RaiseArm(5, 0, 1000), new IntakeOn(IntakeOn.Use.IN)));

        //scheduler.addCommand(new ParallelCommand(new RaiseArm(0,0, RaiseArm.Options.Stacks), new IntakeOn(IntakeOn.Use.IN)));
        //scheduler.addCommand(new RaiseArm());
        //scheduler.addCommand(new waitCommand(1));
        scheduler.addCommand(new ParallelCommand(new RaiseArm(0,0, 1000), new IntakeOff()));
        scheduler.addCommand(new DriveToPosCommand(0.7, 35000));
        scheduler.addCommand(new ParallelCommand(new TurnToAngle(0.5, Math.toRadians(72)), new RaiseArm(4, 0, 3000)));
        scheduler.addCommand(new IntakeOn(IntakeOn.Use.OUT));
        scheduler.addCommand(new IntakeOff());
       // scheduler.addCommand(new waitCommand(1));
        scheduler.addCommand(new RaiseArm(0, 0, 3000));
        //scheduler.addCommand(new TurnToAngle(0.5, Math.toRadians(85)));

        ParkingScheduler1 = new Scheduler(robot, telemetry);
        ParkingScheduler1.addCommand(new DriveToPosCommand(-0.7, -6000));
        ParkingScheduler1.addCommand(new TurnToAngle(0.5, Math.toRadians(-90)));
        ParkingScheduler1.addCommand(new DriveToPosCommand(-0.7, -40000));
        ParkingScheduler1.addCommand(new TurnToAngle(0.5, 0));
        ParkingScheduler1.addCommand(new DriveToPosCommand(-0.9, -10000));

        ParkingScheduler2 = new Scheduler(robot, telemetry);
        //ParkingScheduler2.addCommand(new TurnToAngle(0.5, Math.toRadians(10)));
        ParkingScheduler2.addCommand(new DriveToPosCommand(-0.7, -6000));
        ParkingScheduler2.addCommand(new TurnToAngle(0.5, Math.toRadians(0)));
        ParkingScheduler2.addCommand(new DriveToPosCommand(-0.9, -20000));

        ParkingScheduler3 = new Scheduler(robot, telemetry);
        ParkingScheduler3.addCommand(new DriveToPosCommand(-0.7, -6000));
        //ParkingScheduler3.addCommand(new TurnToAngle(0.5, Math.toRadians(45)));
        ParkingScheduler3.addCommand(new TurnToAngle(0.5, Math.toRadians(90)));
        ParkingScheduler3.addCommand(new DriveToPosCommand(-0.7, -40000));
        ParkingScheduler3.addCommand(new TurnToAngle(0.5, 0));
        ParkingScheduler3.addCommand(new DriveToPosCommand(-0.9, -20000));

        telemetry.addLine("Robot is READY");
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

        while (opModeIsActive()){
            savePositions();
        }

    }

    public void savePositions() {
        try {
            PrintWriter saver = new PrintWriter(new FileOutputStream(AppUtil.ROBOT_DATA_DIR + "position.txt"));

            saver.println(robot.gyro.getHeading());
            saver.println(robot.arm.getAverageFirst());
            saver.println(robot.arm.getAverageSecond());    // turn angle
            saver.println();                             // elevator lower motors encoder
            saver.println();                             // elevator higher motors encoder
            saver.close();

            telemetry.addLine("robot positions saved");
            telemetry.update();
        } catch (Exception e) {
            telemetry.addLine("Error, file not found");
            telemetry.update();
            return;
        }
    }
}

