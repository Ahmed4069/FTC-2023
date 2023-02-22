package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class PositionVisualizer extends OpMode {
    ftclibArm arm;

    @Override
    public void init() {
       arm = new ftclibArm(hardwareMap, telemetry);


       arm.secArmLift2.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
       arm.secArmLift1.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
    }

    @Override
    public void loop() {
        telemetry.addData("first arm", arm.getAverageFirst());
        telemetry.addData("second arm", arm.getAverageSecond());
    }
}
