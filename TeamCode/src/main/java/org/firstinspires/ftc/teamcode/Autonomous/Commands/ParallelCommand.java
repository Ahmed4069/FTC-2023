package org.firstinspires.ftc.teamcode.Autonomous.Commands;

import java.util.Calendar;

public class ParallelCommand extends Commands{

    private Commands command1, command2;
    /**NEVER USE WITH DRIVETOPOSCOMMAND!!!**/
    public ParallelCommand (Commands c1, Commands c2){
        command1 = c1;
        command2 = c2;
    }
    @Override
    public void start() {
        command1.setSubsystems(robot, telemetry);
        command2.setSubsystems(robot, telemetry);

        command1.start();
        command2.start();
    }

    @Override
    public void loop() {
        if(!command1.isFinished()) command1.loop();
        if(!command2.isFinished()) command2.loop();
    }

    @Override
    public boolean isFinished() {
        return command1.isFinished() && command2.isFinished();
    }

}
