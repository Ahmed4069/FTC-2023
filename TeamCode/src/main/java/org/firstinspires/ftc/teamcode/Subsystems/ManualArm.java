package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ManualArm {
    DcMotor armLift1, armLift2, secArmLift1, secArmLift2;

    public ManualArm(HardwareMap hardwareMap){
        armLift1 = hardwareMap.get(DcMotor.class, "lift1");
        armLift2 = hardwareMap.get(DcMotor.class, "lift2");
        secArmLift1 = hardwareMap.get(DcMotor.class, "secArm1");
        secArmLift2 = hardwareMap.get(DcMotor.class, "secArm2");

        armLift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armLift1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //armLift1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //armLift1.setTargetPosition(0);
        //armLift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armLift1.setDirection(DcMotorSimple.Direction.REVERSE);

        armLift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armLift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //armLift2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //armLift2.setTargetPosition(0);
        //armLift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        secArmLift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        secArmLift1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //secArmLift1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //secArmLift1.setTargetPosition(0);
        //secArmLift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        secArmLift1.setDirection(DcMotorSimple.Direction.REVERSE);

        secArmLift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        secArmLift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
       // secArmLift2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //secArmLift2.setTargetPosition(0);
        //secArmLift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void MoveManually(double firstSpeed, double SecondSpeed){
        armLift1.setPower(firstSpeed);
        armLift2.setPower(firstSpeed);
        secArmLift2.setPower(SecondSpeed);
        secArmLift1.setPower(SecondSpeed);
    }
}
