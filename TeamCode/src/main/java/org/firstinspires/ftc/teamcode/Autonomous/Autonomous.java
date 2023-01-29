package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Main.Robot.Robot;
import org.firstinspires.ftc.teamcode.Vision.SignalClassifier;

import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.PrintWriter;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class Autonomous extends LinearOpMode {
    Robot robot;
    SignalClassifier classifier;

/*    public void savePositions() {
        try {
            PrintWriter position = new PrintWriter(new FileOutputStream(AppUtil.ROBOT_DATA_DIR + "position.txt"));
*//*

            position.println(m_drive.position[0]);          // x-value
            position.println(m_drive.position[1]);          // y-value
            position.println(m_drive.getHeading());         // turn angle
            position.println();                             // elevator lower motors encoder
            position.println();                             // elevator higher motors encoder
*//*

            position.close();
*//*
            telemetry.addLine("robot positions saved");
            telemetry.update();
*//*
        } catch (Exception e) {
*//*
            telemetry.addLine("Error, file not found");
            telemetry.update();
*//*
        }
    }*/

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap, telemetry);

        classifier = new SignalClassifier(hardwareMap, telemetry);

        waitForStart();

        boolean complete = false;
//        String signal = classifier.classify();
        String signal = "ConeSignal1";
        classifier.closeSignalClassifier();
        telemetry.addData("signal", signal);
        telemetry.update();

        if(signal == "ConeSignal1"){
            telemetry.addData("in", 1);
            telemetry.update();
            robot.driveBase.setMotorPowers(0.4, 0.4, 1, 1);
            sleep(900);
            robot.driveBase.stop();
            sleep(300);
            robot.driveBase.setMotorPowers(-1, -1, 1, 1);
            sleep(500);
            robot.driveBase.setMotorPowers(0.5, 0.5, 1, 1);
            sleep(3000);
            robot.disable();
            complete = true;
        }
        else if(signal == "ConeSignal3"){
            telemetry.addData("in", 2);
            telemetry.update();
            robot.driveBase.setMotorPowers(1, 1, 0.7, 0.7);
            sleep(2500);
            robot.driveBase.stop();
            sleep(300);
            robot.driveBase.setMotorPowers(1, 1, -1, -1);
            sleep(500);
            robot.driveBase.setMotorPowers(1,1,0.7,0.7);
            sleep(3000);
            robot.disable();
            complete = true;
        }
        else{
            telemetry.addData("in", 3);
            telemetry.update();
            robot.driveBase.setMotorPowers(0.43,0.43,1,1);
            sleep(3000);
            robot.disable();
            complete = true;
        }



        if(complete == true){
            robot.disable();
        }
/*        String signal;
        robot = new Robot(hardwareMap, telemetry);
//        classifier = new SignalClassifier(hardwareMap, telemetry);
//
//        waitForStart();
//
//        signal = classifier.classify();
//        classifier.closeSignalClassifier();

//        telemetry.addData("signal", signal);
//        telemetry.update();

        waitForStart();

        robot.driveBase.turn(50000, 60000);
        while (true) {
            telemetry.addLine("avg error: " + robot.driveBase.loop());
            if (gamepad2.a) {
                break;
            }
            telemetry.update();
        }*/


////        switch (signal) {
//            case "ConeSignal1":
//                robot.driveBase.setMotorPowers(-0.5, 0.5, 0.5, -0.5);
//                sleep(1000);
//                robot.driveBase.stop();
//                sleep(500);
//                break;
//
//            case "ConeSignal3":
//                robot.driveBase.setMotorPowers(0.5, -0.5, -0.5, 0.5);
//                sleep(1000);
//                robot.driveBase.stop();
//                sleep(500);
//                break;
//
//            default:
//                break;
//        }
//
//        robot.driveBase.setMotorPowers(1, 1, 1, 1);
//        sleep(1000);
//        robot.driveBase.stop();

        PrintWriter position = null;
        try {
            position = new PrintWriter(new FileOutputStream(AppUtil.ROBOT_DATA_DIR + "position.txt"));

            position.println(robot.driveBase.getHeading());         // turn angle
            position.close();
        } catch (FileNotFoundException e) {
            e.printStackTrace();
        }


    }
}