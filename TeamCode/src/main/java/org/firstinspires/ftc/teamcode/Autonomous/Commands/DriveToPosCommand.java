package org.firstinspires.ftc.teamcode.Autonomous.Commands;


import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Timer;

public class DriveToPosCommand extends Commands {
    private double speed;
    public int desiredPosition;
    private int initialPos;
    public double straightAngle;

    private double kP = 0.000775, kI = 0, kD = 0.0001;
    private int I = 0;
    private int lasterror;

    ElapsedTime timer = new ElapsedTime();

    public DriveToPosCommand (double speed, int desiredPosition){
        this.speed = speed;
        this.desiredPosition = desiredPosition;
    }

    @Override
    public void start(){
        this.initialPos = robot.driveBase.getLeftEncoderValue();
//        this.initialRight = robot.driveBase.getRightEncoderValue();

        this.lasterror = this.desiredPosition;
//        this.lasterrorright = this.desiredPosition;

        this.straightAngle = robot.gyro.getHeading();
    }

    @Override
    public void loop() {
        int cur = robot.driveBase.getLeftEncoderValue() - this.initialPos;
        int error = this.desiredPosition - cur;

        I += error * timer.seconds();
        double d = (error - lasterror) / timer.seconds();

        lasterror = error;
        timer.reset();

        double power = Math.max(Math.min((error * kP) + (d * kD) + (I * kI), 1), -1) * 0.75;
        robot.driveBase.tankDrive(power + wrapAround(robot.gyro.getHeading(), straightAngle) * 0.55,
                                 power - wrapAround(robot.gyro.getHeading(), straightAngle) * 0.55);

        telemetry.addData("distance", error);
        telemetry.addData("angle", wrapAround(robot.gyro.getHeading(), straightAngle));
    }

    // not sure if this works for all angles yet
    public double wrapAround(double angle_cur, double angle_tar) {
        double angle = angle_cur - angle_tar;
        return Math.abs(angle) > Math.PI? (Math.PI - Math.abs(angle)) * Math.signum(angle) : angle;
    }

    @Override
    public boolean isFinished() {
        return Math.abs(this.desiredPosition - (robot.driveBase.getLeftEncoderValue() - this.initialPos)) < 50;
    }
}
