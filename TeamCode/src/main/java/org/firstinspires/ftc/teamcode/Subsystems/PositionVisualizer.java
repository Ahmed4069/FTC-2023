package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

//@TeleOp
public class PositionVisualizer extends OpMode {
    DcMotor leftMotord, rightMotord;
    DcMotor leftMotorh, rightMotorh;

    @Override
    public void init() {
        leftMotord = hardwareMap.get(DcMotor.class, "lift1");
        rightMotord = hardwareMap.get(DcMotor.class, "lift2");
        leftMotorh = hardwareMap.get(DcMotor.class, "hexmotor1");
        rightMotorh = hardwareMap.get(DcMotor.class, "hexmotor2");

        leftMotord.setDirection(DcMotorSimple.Direction.REVERSE);

        rightMotorh.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotord.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotorh.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotord.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftMotord.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {
        telemetry.addData("left lower", leftMotord.getCurrentPosition());
        telemetry.addData("right lower", rightMotord.getCurrentPosition());
        telemetry.addData("left upper", leftMotorh.getCurrentPosition());
        telemetry.addData("right upper", rightMotorh.getCurrentPosition());
        telemetry.update();
    }
}
