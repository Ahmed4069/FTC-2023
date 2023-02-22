package org.firstinspires.ftc.teamcode.Autonomous.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Autonomous.Commands.DriveToPosCommand;
import org.firstinspires.ftc.teamcode.Autonomous.Commands.RaiseArm;
import org.firstinspires.ftc.teamcode.Autonomous.Scheduler;
import org.firstinspires.ftc.teamcode.Main.Robot.Robot;

@Autonomous
public class raiseArm extends LinearOpMode {
    Scheduler scheduler;
    Robot robot;
    boolean done = false;
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap, telemetry);
        scheduler = new Scheduler(robot, telemetry);

        //scheduler.addCommand(new DriveToPosCommand(0.5, 1000));
        scheduler.addCommand(new RaiseArm(4, 0, 0 ));

        waitForStart();

        while(opModeIsActive() && !done){
            scheduler.run();
            telemetry.update();
            idle();

            if(scheduler.getListSize() == 0){
                done = true;
                break;
            }
        }

    }

}
