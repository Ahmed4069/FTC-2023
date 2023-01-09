package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Autonomous.Commands.RaiseArm;
import org.firstinspires.ftc.teamcode.Main.Robot.Robot;

@Autonomous
public class AutoRight extends LinearOpMode{
    Robot robot;
    Scheduler scheduler;


    @Override
    public void runOpMode() {
        robot = new Robot(hardwareMap, telemetry);
        scheduler = new Scheduler(robot, telemetry);

//        scheduler.addCommand(new DriveToPosCommand(0.5, 85000));
//        scheduler.addCommand(new TurnToAngle(0.5, Math.PI / 4));
//        scheduler.addCommand(new DriveToPosCommand(0.5, 5000));
        scheduler.addCommand(new RaiseArm(4, 0, RaiseArm.Options.Junction));
//        scheduler.addCommand(new IntakeOn(IntakeOn.Use.OUT));

        waitForStart();

        while(opModeIsActive() && scheduler.getListSize() != 0) {
            scheduler.run();
            telemetry.update();
            idle();
        }

        telemetry.addLine("done");
        telemetry.addData("difference", robot.arm.diff());
        telemetry.update();

        while (opModeIsActive()) {
        }

    }
}
