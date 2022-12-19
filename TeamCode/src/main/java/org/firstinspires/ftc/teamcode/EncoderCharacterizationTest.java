package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class EncoderCharacterizationTest extends OpMode {
    DcMotor motorRF, motorLF;
    double rightencoder, leftencoder;
    @Override
    public void init() {
        motorLF = hardwareMap.get(DcMotor.class, "motorLF");
        motorRF = hardwareMap.get(DcMotor.class, "motorRF");

        motorRF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorLF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    @Override
    public void loop() {
        rightencoder = motorRF.getCurrentPosition();
        leftencoder = motorLF.getCurrentPosition();

        telemetry.addData("right: ", rightencoder);
        telemetry.addData("left: ", leftencoder);
        telemetry.update();
    }
}
