package org.firstinspires.ftc.teamcode.Autonomous.Commands;

import com.arcrobotics.ftclib.controller.wpilibcontroller.RamseteController;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.DifferentialDriveKinematics;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.DifferentialDriveWheelSpeeds;
import com.arcrobotics.ftclib.trajectory.Trajectory;
import com.arcrobotics.ftclib.trajectory.TrajectoryConfig;
import com.arcrobotics.ftclib.trajectory.TrajectoryGenerator;
import com.arcrobotics.ftclib.util.Timing;

import java.util.ArrayList;
import java.util.concurrent.TimeUnit;

public class TrajectoryFollower extends Commands{
    TrajectoryConfig config;
    Pose2d start;
    ArrayList<Translation2d> interiorWaypoints;
    Pose2d end;
    Trajectory trajectory;
    RamseteController controller;
    Pose2d currentPose;
    DifferentialDriveKinematics kinematics;
    private double trackWidth = 0.4064;
    private final double b = 2, zeta = 0.7;
    double startingTime = 0;
    double elapsedTime = 0;
    Timing.Timer timer;

    public TrajectoryFollower(Pose2d start, ArrayList<Translation2d> interiorWaypoints, Pose2d end, boolean isReverse) {
        config = new TrajectoryConfig(1, 0.5);
        config.setReversed(isReverse);
        this.start = start;
        this.interiorWaypoints = interiorWaypoints;
        this.end = end;
        trajectory = TrajectoryGenerator.generateTrajectory(start, interiorWaypoints, end, config);
        controller = new RamseteController(b, zeta);
        kinematics = new DifferentialDriveKinematics(trackWidth);

        timer = new Timing.Timer(30, TimeUnit.SECONDS);
    }

    @Override
    public void start(){
        timer.start();
        startingTime = timer.elapsedTime();
    }

    @Override
    public void loop() {
        elapsedTime = timer.elapsedTime() - startingTime;

        robot.getDriveBase().updatePosition();
        currentPose = robot.getDriveBase().getPose();

        ChassisSpeeds speeds = controller.calculate(currentPose, trajectory.sample(elapsedTime));

        DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(speeds);

        robot.getDriveBase().tankDrive(wheelSpeeds.leftMetersPerSecond, wheelSpeeds.rightMetersPerSecond);
    }

    @Override
    public boolean isFinished() {
        Pose2d poseDifference = robot.getDriveBase().getPose().relativeTo(end);
        return (Math.abs(poseDifference.getX()) < 0.2) && (Math.abs(poseDifference.getY()) < 0.2) || elapsedTime + 1 > trajectory.getTotalTimeSeconds();
    }
}
