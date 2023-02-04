package org.firstinspires.ftc.teamcode.Autonomous.Commands;

import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.TimeUnit;

public class waitCommand extends Commands{
    Timing.Timer timer;
    long time;

    public waitCommand(long time){
        this.time = time;
        timer = new Timing.Timer(time, TimeUnit.SECONDS);
    }
    @Override
    public void start() {
        timer.start();
    }

    @Override
    public void loop() {
        telemetry.addData("Remaining time in wait", timer.elapsedTime());
    }

    @Override
    public boolean isFinished() {
        return timer.done();
    }
}
