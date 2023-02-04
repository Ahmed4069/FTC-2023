package org.firstinspires.ftc.teamcode.Autonomous.Commands;

import com.qualcomm.robotcore.util.ElapsedTime;

public class waitCommand extends Commands{
    ElapsedTime timer;
    double time;

    public waitCommand(double time){
        this.time = time;
        timer = new ElapsedTime();
    }
    @Override
    public void start() {
        timer.startTime();
    }

    @Override
    public void loop() {

    }

    @Override
    public boolean isFinished() {
        return timer.seconds() >= this.time;
    }
}
