package org.firstinspires.ftc.teamcode.Autonomous.Commands;

import com.qualcomm.robotcore.util.ElapsedTime;

public class IntakeOn extends Commands {

    double speed;
    //ElapsedTime timer = new ElapsedTime();

    public IntakeOn(Use use){
        this.speed = (use == Use.IN) ? 1 : -1;
    }

    @Override
    public void start() {
        //timer.startTime();
    }

    @Override
    public void loop() {
        robot.intake.setServo(speed);
    }

    @Override
    public boolean isFinsihed() {
        return robot.intake.getSpeed() == 1 || robot.intake.getSpeed() == -1;
    }

    public enum Use{
        IN,
        OUT
    }


}
