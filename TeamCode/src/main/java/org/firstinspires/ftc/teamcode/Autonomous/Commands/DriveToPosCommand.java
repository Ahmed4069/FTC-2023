package org.firstinspires.ftc.teamcode.Autonomous.Commands;


public class DriveToPosCommand extends Commands {
    private double speed;
    public double desiredPosition;
    private int initialLeft, initialRight;
    public double straightAngle;

    public DriveToPosCommand (double speed, double desiredPosition){
        this.speed = speed;
        this.desiredPosition = desiredPosition;
        this.straightAngle = robot.gyro.getHeading();
    }

    @Override
    public void start(){
        this.initialLeft = robot.driveBase.getLeftEncoderValue();
        this.initialRight = robot.driveBase.getRightEncoderValue();
    }

    @Override
    public void loop() {
        double leftSpeed = speed - ((robot.gyro.getHeading() - straightAngle) * 0.2);
        double rightSpeed = speed + ((robot.gyro.getHeading() - straightAngle) * 0.2);

        robot.driveBase.setMotorPowers(leftSpeed, leftSpeed, rightSpeed, rightSpeed);
    }

    @Override
    public boolean isFinsihed() {
        telemetry.addData("position", (robot.driveBase.getLeftEncoderValue() + robot.driveBase.getRightEncoderValue() - (this.initialLeft + this.initialRight)));
        return this.desiredPosition - (robot.driveBase.getLeftEncoderValue() + robot.driveBase.getRightEncoderValue() - (this.initialLeft + this.initialRight)) / 2 < 100;
    }
}
