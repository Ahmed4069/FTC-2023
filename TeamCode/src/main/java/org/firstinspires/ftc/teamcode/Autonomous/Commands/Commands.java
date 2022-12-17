package org.firstinspires.ftc.teamcode.Autonomous.Commands;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Main.Robot.Robot;

public abstract class Commands {
    protected Robot robot;
    protected Telemetry telemetry;

    public void setSubsystems(Robot robot, Telemetry telemetry){
        this.robot = robot;
        this.telemetry = telemetry;
    }

    public abstract void start();
    public abstract void loop();
    public abstract boolean isFinsihed();
}
