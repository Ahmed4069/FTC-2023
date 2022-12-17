package org.firstinspires.ftc.teamcode.Autonomous.Commands;


public class DriveToPosCommand extends Commands {
    private double speed;
    public double desiredPosition;
    private double initialPosition;

    public DriveToPosCommand (double speed, double desiredPosition){
        this.speed = speed;
        this.desiredPosition = desiredPosition;
    }

    @Override
    public void start(){
        this.initialPosition = robot.driveBase.getEncoder();
    }

    @Override
    public void loop() {
        robot.driveBase.setMotorPowers(speed, speed, speed, speed);
        telemetry.addData("Desired Position", desiredPosition);
        telemetry.addData("Starting Position", initialPosition);
    }

    @Override
    public boolean isFinsihed() {
        return Math.abs(desiredPosition - initialPosition) < Math.abs(robot.driveBase.getHeading());
    }
}
