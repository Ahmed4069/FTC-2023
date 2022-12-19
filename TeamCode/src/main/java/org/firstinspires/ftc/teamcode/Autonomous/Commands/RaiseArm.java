package org.firstinspires.ftc.teamcode.Autonomous.Commands;

public class RaiseArm extends Commands {
    public enum Heights {
        MEDIUM,
        HIGH
    }

    // the index of the encoder values in the position-to-encoder map
    int desired_height;

    // the desired power
    double desired_power;

    public RaiseArm (Heights h, double p) {
        this.desired_height = (h == Heights.MEDIUM) ? 2 : 3;
        this.desired_power = p;
    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {

    }

    @Override
    public boolean isFinsihed() {
        return false;
    }
}
