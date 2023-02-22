package org.firstinspires.ftc.teamcode.Subsystems;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class manualArm {
    DcMotor secArmLift1, secArmLift2, armLift1, armLift2;

    public manualArm(HardwareMap hardwareMap){
        secArmLift1 = hardwareMap.get(DcMotor.class, "secArm1");
        secArmLift2 = hardwareMap.get(DcMotor.class, "secArm2");
        armLift1 = hardwareMap.get(DcMotor.class, "lift1");
        armLift2 = hardwareMap.get(DcMotor.class, "lift2");

        armLift1.setZeroPowerBehavior(BRAKE);
        armLift2.setZeroPowerBehavior(BRAKE);
        secArmLift1.setZeroPowerBehavior(BRAKE);
        secArmLift2.setZeroPowerBehavior(BRAKE);
    }

    public void update(double speed1, double speed2){
        armLift1.setPower(speed1);
        armLift2.setPower(speed1);

        secArmLift1.setPower(speed2);
        secArmLift2.setPower(speed2);
    }
}
