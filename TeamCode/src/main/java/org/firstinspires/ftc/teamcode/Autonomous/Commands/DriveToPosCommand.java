package org.firstinspires.ftc.teamcode.Autonomous.Commands;


import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Timer;

public class DriveToPosCommand extends Commands {
    private double speed;
    public int desiredPosition;
    private int initialLeft, initialRight;
    public double straightAngle;

    private double kP = 0.01, kI = 0, kD = 0;
    private int Ileft = 0, Iright = 0;
    private int lasterrorleft, lasterrorright;

    ElapsedTime timer = new ElapsedTime();

    public DriveToPosCommand (double speed, int desiredPosition){
        this.speed = speed;
        this.desiredPosition = desiredPosition;
    }

    @Override
    public void start(){
        this.initialLeft = robot.driveBase.getLeftEncoderValue();
        this.initialRight = robot.driveBase.getRightEncoderValue();

        this.lasterrorleft = this.desiredPosition;
        this.lasterrorright = this.desiredPosition;

        this.straightAngle = robot.gyro.getHeading();
    }

    @Override
    public void loop() {
        int curleft = robot.driveBase.getLeftEncoderValue() - this.initialLeft,
            curright = robot.driveBase.getRightEncoderValue() - this.initialRight;

        int errorleft = this.desiredPosition - curleft,
            errorright = this.desiredPosition - curright;

        Ileft += errorleft * timer.seconds();
        Iright += errorright * timer.seconds();

        double d_l = (errorleft - lasterrorleft) * timer.seconds(),
               d_r = (errorright - lasterrorright) * timer.seconds();

        lasterrorleft = errorleft;
        lasterrorright = errorright;

        timer.reset();

        robot.driveBase.tankDrive((errorright * kP) + (d_r * kD) + (Iright * kI),
                                  (errorleft * kP) + (d_l * kD) + (Ileft * kI));

//        double leftSpeed = speed - ((robot.gyro.getHeading() - straightAngle) * 0.2);
//        double rightSpeed = speed + ((robot.gyro.getHeading() - straightAngle) * 0.2);
//
//        robot.driveBase.setMotorPowers(leftSpeed, leftSpeed, rightSpeed, rightSpeed);
        telemetry.addData("left: ", curleft);
        telemetry.addData("right:", curright);
    }

    @Override
    public boolean isFinsihed() {
//        telemetry.addData("position", (robot.driveBase.getLeftEncoderValue() + robot.driveBase.getRightEncoderValue() - (this.initialLeft + this.initialRight)));
//        return Math.abs(this.desiredPosition - (robot.driveBase.getLeftEncoderValue() + robot.driveBase.getRightEncoderValue() - (this.initialLeft + this.initialRight)) / 2) < 100;
        return false;
    }
}
