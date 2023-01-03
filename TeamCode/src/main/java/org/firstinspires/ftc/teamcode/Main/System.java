package org.firstinspires.ftc.teamcode.Main;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class System {
    Telemetry telemetry;

    public System (Telemetry telemetry){
        this.telemetry = telemetry;
    }

    public void printString(String label, String msg){
        telemetry.addData(label, msg);
    }

    public void printInt(String label, int msg){
        telemetry.addData(label, msg);
    }

    public void printDouble(String label, double msg){
        telemetry.addData(label, msg);
    }

    public void printBoolean(String label, boolean msg){
        telemetry.addData(label, msg);
    }

    public double convertUnits (double value, Units unit){
        if (unit == Units.INCHES) return value / Constants.encoderToInchesConstant;
        else return value / Constants.encoderToFeetConstant;
    }

    public enum Units{
        INCHES,
        FEET
    }
}


