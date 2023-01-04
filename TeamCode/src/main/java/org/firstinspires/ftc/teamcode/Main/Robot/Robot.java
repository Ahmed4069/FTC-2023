package org.firstinspires.ftc.teamcode.Main.Robot;

import android.sax.TextElementListener;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Main.System;
import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.DriveBase;
import org.firstinspires.ftc.teamcode.Subsystems.Gyro;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;

public class Robot {
    public DriveBase driveBase;
    public Intake intake;
    public Arm arm;
    public Gyro gyro;
    public System System;

    public Robot(HardwareMap hardwareMap, Telemetry tele){
        driveBase = new DriveBase(hardwareMap, tele);
        intake = new Intake(hardwareMap);
        arm = new Arm(hardwareMap, tele);
        gyro = new Gyro(hardwareMap);
        System = new System(tele);
    }

    public void disable(){
        intake.disable();
        arm.disable();
        driveBase.disable();
    }
}
