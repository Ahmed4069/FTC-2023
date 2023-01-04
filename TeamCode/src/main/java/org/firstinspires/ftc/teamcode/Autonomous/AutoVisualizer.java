package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Main.Robot.Robot;

@Autonomous
public class AutoVisualizer extends OpMode {
    Robot robot;

    @Override
    public void init() {
        robot = new Robot(hardwareMap, telemetry);
    }

    @Override
    public void loop() {
        robot.System.printDouble("Left Encoder: ", robot.driveBase.getLeftEncoderValue());
        robot.System.printDouble("Right Encoder: ", robot.driveBase.getRightEncoderValue());

        robot.System.printDouble("Gyro Heading: ", robot.gyro.getHeading());
    }

}
