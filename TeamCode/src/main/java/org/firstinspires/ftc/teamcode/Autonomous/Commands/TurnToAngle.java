package org.firstinspires.ftc.teamcode.Autonomous.Commands;

import org.firstinspires.ftc.teamcode.Commands;
import org.firstinspires.ftc.teamcode.Main.Robot.Robot;

public class TurnToAngle extends Commands {
    private double currentAngle;
    public double desiredAngle;
    private double angDeg;
    private double speed;
    private double sign;

    public TurnToAngle(double speed, double desiredAngle){
        this.desiredAngle = desiredAngle;
        this.speed = speed;
    }

    @Override
    public void start() {
        this.currentAngle = robot.gyro.getHeading();
        angDeg = Math.toDegrees(currentAngle);
        sign = Math.signum(desiredAngle - angDeg);
    }

    @Override
    public void loop() {
        robot.driveBase.setMotorPowers(-speed + sign, -speed + sign, speed +sign, speed + sign);
    }

    @Override
    public boolean isFinsihed() {
        return Math.abs(robot.gyro.getHeading()) < 2.5;
    }
}
