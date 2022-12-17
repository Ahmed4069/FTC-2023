package org.firstinspires.ftc.teamcode.Main.Robot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Subsystems.Gyro;

public class headingPrinter extends OpMode {
    public Gyro gyro;

    @Override
    public void init() {
        gyro = new Gyro(hardwareMap);
    }

    @Override
    public void loop() {
        telemetry.addData("heading", gyro.getHeading());
        telemetry.update();
    }
}
