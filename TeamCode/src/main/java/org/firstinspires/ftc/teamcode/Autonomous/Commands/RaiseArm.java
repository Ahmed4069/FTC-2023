package org.firstinspires.ftc.teamcode.Autonomous.Commands;

import android.graphics.Path;

import com.arcrobotics.ftclib.util.Timing;

import java.util.concurrent.TimeUnit;

public class RaiseArm extends Commands {
//    public enum Options{
//        Junction,
//        Stacks
//    }

    // the index of the encoder values in the position-to-encoder map
    int desired_height;

    // the desired power
    double desired_power;
    //Options options;
    Timing.Timer timer;

    public RaiseArm (int h, double p, long times) {
        this.desired_height = h;
        this.desired_power = p;
        long time = times;
        timer = new Timing.Timer(time, TimeUnit.MILLISECONDS);
    }

    @Override
    public void start() {
        timer.start();
        //robot.getArm().moveArmToHeightOfJunction(desired_height);
        telemetry.addData("secArm pos", robot.arm.getAverageSecond());
        //telemetry.addData("target", robot.arm.requiredAnglesforClearence[robot.arm.lastPos][1]);
//        if (options == Options.Junction){
//            robot.arm.moveArmToHeightOfJunction(desired_height);
//        }
//        else if (options == Options.Stacks){
//            robot.arm.moveArmToHeightOfStacks();
//        }
    }

    @Override
    public void loop() {
         telemetry.addData("looped", desired_height);
         telemetry.addData("secArm pos", robot.arm.getAverageSecond());
         telemetry.update();

        // if (options == Options.Junction){
             robot.getArm().moveArmToHeightOfJunction(desired_height, 0, 0);

         //}
         //else if (options == Options.Stacks){
          //   robot.arm.moveArmToHeightOfStacks();
        // }
    }

    @Override
    public boolean isFinished() {
        if (timer.done()){
            robot.getArm().stopFirstArm();
            robot.getArm().stopSecondArm();
            return true;
        }
        else{
            return false;
        }
    }
}
