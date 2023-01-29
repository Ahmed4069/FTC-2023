package org.firstinspires.ftc.teamcode.Autonomous.Commands;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

public class ChangeColourCommand extends Commands{
    RevBlinkinLedDriver.BlinkinPattern colour;

    public ChangeColourCommand(RevBlinkinLedDriver.BlinkinPattern colour){
         this.colour= colour;
    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {
        robot.blinkin.update(colour);
    }

    @Override
    public boolean isFinsihed() {
        return true;
    }
}
