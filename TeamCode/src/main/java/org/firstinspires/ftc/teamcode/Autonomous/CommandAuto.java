package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Autonomous.Commands.DriveToPosCommand;
import org.firstinspires.ftc.teamcode.Autonomous.Commands.IntakeOff;
import org.firstinspires.ftc.teamcode.Autonomous.Commands.IntakeOn;
import org.firstinspires.ftc.teamcode.Autonomous.Commands.TurnToAngle;
import org.firstinspires.ftc.teamcode.Main.Robot.Robot;
import org.firstinspires.ftc.teamcode.Vision.SignalClassifier;

@Autonomous
public class CommandAuto extends LinearOpMode{
    Robot robot;
    Scheduler scheduler;
    SignalClassifier classifier;
    boolean clearToStartAuto = false;

    @Override
    public void runOpMode() {
        robot = new Robot(hardwareMap, telemetry);
        scheduler = new Scheduler(robot, telemetry);
        classifier = new SignalClassifier(hardwareMap, telemetry);

        scheduler.addCommand(new DriveToPosCommand(0.5, 3000));
        scheduler.addCommand(new TurnToAngle(0.5, 30));
        //lift Arm
        //arm = x
        scheduler.addCommand(new IntakeOn(IntakeOn.Use.OUT));
        scheduler.addCommand(new IntakeOff());
        //arm down
        scheduler.addCommand(new TurnToAngle(0.5, -90));
        scheduler.addCommand(new DriveToPosCommand(0.5, 1500));
        scheduler.addCommand(new TurnToAngle(0.5, 0));
        scheduler.addCommand(new DriveToPosCommand(0.5, 1000));
        scheduler.addCommand(new TurnToAngle(0.5, 90));
        scheduler.addCommand(new DriveToPosCommand(-0.5, 1000));
        //get cone from cone stack
        scheduler.addCommand(new IntakeOn(IntakeOn.Use.IN));
        scheduler.addCommand(new IntakeOff());

        scheduler.addCommand(new DriveToPosCommand(0.5, 1500));
        scheduler.addCommand(new TurnToAngle(0.5, 60));
        scheduler.addCommand(new DriveToPosCommand(0.2, 100));
        //lift and deposit cone
        scheduler.addCommand(new IntakeOn(IntakeOn.Use.OUT));

        waitForStart();


        if(clearToStartAuto){
//            String signal = classifier.classify();
            String signal = "";

            robot.System.printString("Signal: ", signal);

            if(signal == "ConeSignal1"){
                scheduler.addCommand(new DriveToPosCommand(-0.5, 500));
                scheduler.addCommand(new TurnToAngle(0.5, 0));
                scheduler.addCommand(new DriveToPosCommand(-0.5, 1000));
                scheduler.addCommand(new TurnToAngle(0.5, -90));
                scheduler.addCommand(new DriveToPosCommand(0.5, 500));
            }
            else if (signal == "ConeSignal2"){
                scheduler.addCommand(new DriveToPosCommand(-0.5, 500));
                scheduler.addCommand(new TurnToAngle(0.5, 0));
                scheduler.addCommand(new DriveToPosCommand(-0.5, 1000));
            }
            else if (signal == "ConeSignal3"){
                scheduler.addCommand(new DriveToPosCommand(-0.5, 500));
                scheduler.addCommand(new TurnToAngle(0.5, 0));
                scheduler.addCommand(new DriveToPosCommand(-0.5, 1000));
                scheduler.addCommand(new TurnToAngle(0.5, 90));
                scheduler.addCommand(new DriveToPosCommand(0.5, 500));
            }

            while(opModeIsActive() && scheduler.getListSize() != 0) {
                scheduler.run();
                telemetry.update();
                idle();
            }
        }


    }
}
