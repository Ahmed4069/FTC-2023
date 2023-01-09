package org.firstinspires.ftc.teamcode.Autonomous.Commands;

import android.graphics.Path;

public class RaiseArm extends Commands {
    public enum Options{
        Junction,
        Stacks
    }

    // the index of the encoder values in the position-to-encoder map
    int desired_height;

    // the desired power
    double desired_power;
    Options options;

    public RaiseArm (int h, double p, Options options) {
        this.options = options;
        this.desired_height = h;
        this.desired_power = p;
    }

    @Override
    public void start() {
        if (options == Options.Junction){
            robot.arm.moveArmToHeightOfJunction(desired_height);
        }
        else if (options == Options.Stacks){
            robot.arm.moveArmToHeightOfStacks();
        }
    }

    @Override
    public void loop() {
        robot.arm.moveArmToHeightOfJunction(desired_height);

        // telemetry.addData("looped", desired_height);
        // telemetry.update();

        // if (options == Options.Junction){
        //     robot.arm.moveArmToHeightOfJunction(desired_height);
        // }
        // else if (options == Options.Stacks){
        //     robot.arm.moveArmToHeightOfStacks();
        // }
    }

    @Override
    public boolean isFinsihed() {
        /** warning: this may cause problems later */
        if (robot.arm.atPosition()) {
            telemetry.addData("difference", robot.arm.diff());
            robot.arm.stopFirstArm();
            robot.arm.stopSecondArm();

            return true;
        } else {
            return false;
        }
    }
}
