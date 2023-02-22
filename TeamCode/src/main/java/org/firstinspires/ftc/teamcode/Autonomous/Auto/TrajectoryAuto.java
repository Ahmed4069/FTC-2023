package org.firstinspires.ftc.teamcode.Autonomous.Auto;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Autonomous.Commands.TrajectoryFollower;
import org.firstinspires.ftc.teamcode.Autonomous.Scheduler;
import org.firstinspires.ftc.teamcode.Main.Robot.Robot;

import java.util.ArrayList;

//@Autonomous
public class TrajectoryAuto extends LinearOpMode {
    Scheduler scheduler;
    Robot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap, telemetry);
        scheduler = new Scheduler(robot, telemetry);

        ArrayList<Translation2d> interiorWaypoints = new ArrayList<>();
        interiorWaypoints.add(0, new Translation2d(0, 0.7));

        Pose2d start = robot.getDriveBase().getPose();
        Translation2d endPoint = new Translation2d(-0.5, 1);
        Rotation2d angle = new Rotation2d(Math.toRadians(90));
        Pose2d end = new Pose2d(endPoint, angle);

        scheduler.addCommand(new TrajectoryFollower(start, interiorWaypoints, end, true));

        waitForStart();

        while (opModeIsActive() && scheduler.getListSize() !=0){
            scheduler.run();
            telemetry.update();
            idle();
        }
    }
}
