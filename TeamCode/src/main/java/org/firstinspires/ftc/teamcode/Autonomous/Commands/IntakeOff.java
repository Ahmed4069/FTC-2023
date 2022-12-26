package org.firstinspires.ftc.teamcode.Autonomous.Commands;

public class IntakeOff extends Commands{

    @Override
    public void start() {
        robot.intake.setServo(0);
    }

    @Override
    public void loop() {
        robot.intake.setServo(0);
    }

    @Override
    public boolean isFinsihed() {
        return true;
    }
}
