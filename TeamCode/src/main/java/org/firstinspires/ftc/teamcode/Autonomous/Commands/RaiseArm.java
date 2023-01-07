package org.firstinspires.ftc.teamcode.Autonomous.Commands;

public class RaiseArm extends Commands {

    // the index of the encoder values in the position-to-encoder map
    int desired_height;

    // the desired power
    double desired_power;

    public RaiseArm (int h, double p) {
        this.desired_height = h;
        this.desired_power = p;
    }

    @Override
    public void start() {
        robot.arm.moveArmToHeightOfJunction(desired_height);
    }

    @Override
    public void loop() {
        telemetry.addData("looped", desired_height);
        telemetry.update();
        robot.arm.moveArmToHeightOfJunction(desired_height);
    }

    @Override
    public boolean isFinsihed() {
        /** warning: this may cause problems later */
        return robot.arm.atPosition();
    }
}
