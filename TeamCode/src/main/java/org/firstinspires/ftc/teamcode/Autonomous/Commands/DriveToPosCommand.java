package org.firstinspires.ftc.teamcode.Autonomous.Commands;


public class DriveToPosCommand extends Commands {
    private double speed;
    public double desiredPosition;
    private int initialLeft, initialRight;

    public DriveToPosCommand (double speed, double desiredPosition){
        this.speed = speed;
        this.desiredPosition = desiredPosition;
    }

    @Override
    public void start(){
        this.initialLeft = robot.driveBase.getLeftEncoderValue();
        this.initialRight = robot.driveBase.getRightEncoderValue();
    }

    @Override
    public void loop() {
        robot.driveBase.setMotorPowers(speed, speed, speed, speed);

    }

    @Override
    public boolean isFinsihed() {
        telemetry.addData("position", (robot.driveBase.getLeftEncoderValue() + robot.driveBase.getRightEncoderValue() - (this.initialLeft + this.initialRight)));
        return this.desiredPosition - (robot.driveBase.getLeftEncoderValue() + robot.driveBase.getRightEncoderValue() - (this.initialLeft + this.initialRight)) / 2 < 100;
    }
}
