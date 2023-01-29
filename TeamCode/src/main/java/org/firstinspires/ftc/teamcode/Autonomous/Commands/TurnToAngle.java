package org.firstinspires.ftc.teamcode.Autonomous.Commands;

import com.qualcomm.robotcore.util.ElapsedTime;

public class TurnToAngle extends Commands {
    private double currentAngle;
    public double desiredAngle;
    private double angDeg;
    private double speed;
    private double sign;

    ElapsedTime timer = new ElapsedTime();

    double I = 0, lasterror;
    double kP = 1.01, kD = 0, kI = 0;

    public TurnToAngle(double speed, double desiredAngle){
        this.desiredAngle = desiredAngle;
        this.speed = speed;
    }

    @Override
    public void start() {
        lasterror = wrapAround(this.desiredAngle, robot.gyro.getHeading());
    }

    @Override
    public void loop() {
        double error = wrapAround(this.desiredAngle, robot.gyro.getHeading());

        I += Math.abs(error) * timer.seconds();
        double d = (error - lasterror) / timer.seconds();

        lasterror = error;
        timer.reset();

        double power = (error * kP) + (d * kD) + (I * kI);
        robot.driveBase.tankDrive(-power, power);
    }

    // not sure if this works for all angles yet
    public double wrapAround(double angle_cur, double angle_tar) {
        double angle = angle_cur - angle_tar;
        return Math.abs(angle) > Math.PI? (Math.PI - Math.abs(angle)) * Math.signum(angle) : angle;
    }

    @Override
    public boolean isFinsihed() {
        return Math.abs(wrapAround(this.desiredAngle, robot.gyro.getHeading())) < 0.15;
//        return false;
    }
}
