package org.firstinspires.ftc.teamcode.Main;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Arm;

@TeleOp
public class test2 extends OpMode {
    Arm arm;

    @Override
    public void init() {
        arm = new Arm(hardwareMap, telemetry);
    }

    @Override
    public void loop() {
        arm.moveArmToHeightOfJunction(4);

    }
}
