package org.firstinspires.ftc.teamcode.Autonomous.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Autonomous.Commands.DriveToPosCommand;
import org.firstinspires.ftc.teamcode.Autonomous.Commands.IntakeOn;
import org.firstinspires.ftc.teamcode.Autonomous.Commands.ParallelCommand;
import org.firstinspires.ftc.teamcode.Autonomous.Commands.RaiseArm;
import org.firstinspires.ftc.teamcode.Autonomous.Commands.TurnToAngle;
import org.firstinspires.ftc.teamcode.Autonomous.Commands.waitCommand;
import org.firstinspires.ftc.teamcode.Autonomous.Scheduler;
import org.firstinspires.ftc.teamcode.Main.Robot.Robot;
import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.ftclibArm;

@Autonomous
public class test extends LinearOpMode {
    Scheduler scheduler;
    Robot robot;
    ftclibArm arm;
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap, telemetry);
        scheduler = new Scheduler(robot, telemetry);

        scheduler.addCommand(new RaiseArm(4, 0, 0));
        scheduler.addCommand(new waitCommand(2));
        scheduler.addCommand(new ParallelCommand(new TurnToAngle(0.5, Math.toRadians(90)), new RaiseArm(0,0, 0)));

       waitForStart();

       while(opModeIsActive() && scheduler.getListSize()!=0){
           scheduler.run();
           telemetry.update();
           idle();
       }
    }
}
