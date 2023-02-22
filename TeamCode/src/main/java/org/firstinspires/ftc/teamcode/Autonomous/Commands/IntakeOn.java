package org.firstinspires.ftc.teamcode.Autonomous.Commands;

import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.TimeUnit;

public class IntakeOn extends Commands {

    double speed;
    Timing.Timer timer;

    public IntakeOn(Use use){
        this.speed = (use == Use.IN) ? -1 : 1;
        timer = new Timing.Timer(1, TimeUnit.SECONDS);
    }

    @Override
    public void start() {
        timer.start();
    }

    @Override
    public void loop() {
        robot.intake.setServo(speed);
    }

    @Override
    public boolean isFinished() {
        if (timer.done()){
            robot.intake.setServo(0);
            return true;
        }
        else return false;
    }

    public enum Use{
        IN,
        OUT
    }


}
