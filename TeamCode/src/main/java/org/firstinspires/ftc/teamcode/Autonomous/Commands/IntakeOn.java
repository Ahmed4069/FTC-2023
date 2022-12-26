package org.firstinspires.ftc.teamcode.Autonomous.Commands;

public class IntakeOn extends Commands {

    double speed;

    public IntakeOn(Use use){
        this.speed = (use == Use.IN) ? -1 : 1;
    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {
        robot.intake.setServo(speed);
    }

    @Override
    public boolean isFinsihed() {
        return robot.intake.getSpeed() == speed;
    }

    public enum Use{
        IN,
        OUT
    }


}
