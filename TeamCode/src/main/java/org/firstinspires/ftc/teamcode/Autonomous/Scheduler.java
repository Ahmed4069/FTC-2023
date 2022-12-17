package org.firstinspires.ftc.teamcode.Autonomous;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Commands;
import org.firstinspires.ftc.teamcode.Main.Robot.Robot;

import java.util.ArrayList;
import java.util.List;

public class Scheduler {
    List<Commands> commands = new ArrayList<Commands>();
    Telemetry telemetry;
    private Robot robot;
    boolean started = false;


    public Scheduler(Robot robot, Telemetry telemetry){
        this.robot = robot;
        this.telemetry = telemetry;
    }

    public void addCommand(Commands command) {
        commands.add(command);
        command.setSubsystems(robot, telemetry);
    }
    public void setCommands(int index, Commands command){
        commands.add(index, command);
    }
    public int getListSize(){
        return commands.size();
    }

    public void run(){
        telemetry.addData("Amount of commands", getListSize());

        if(getListSize() != 0){
            Commands nextCommand = commands.get(0);

            if (!started){
                nextCommand.start();
                started = true;
            }

            nextCommand.loop();
            if (nextCommand.isFinsihed()){
                robot.driveBase.disable();
                commands.remove(0);
                telemetry.addData("Command", "removed");
                telemetry.update();
                if(commands.size() != 0) {
                    commands.get(0).start();
                }
            }
        }
    }
    public void disableSubsystems() {
        robot.disable();
    }
}
