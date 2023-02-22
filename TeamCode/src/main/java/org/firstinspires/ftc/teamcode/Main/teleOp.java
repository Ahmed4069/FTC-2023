package org.firstinspires.ftc.teamcode.Main;

//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Subsystems.DriveBase;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.ftclibArm;
import org.firstinspires.ftc.teamcode.Subsystems.manualArm;

import java.io.BufferedInputStream;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.util.Scanner;


/* This is the main object which will control other objects */
@TeleOp
public class teleOp extends OpMode {
    DriveBase m_drive;
    Intake intake;
    /** I changed this*/ ftclibArm arm;

    manualArm manualArm;
    //Encoder encoder;

    RobotMode mode;
    double y, x, rx, triggers;

    int posCode = 0, conesTakenFromStacks = 0;

    boolean manualOverrideActive = false;
    boolean isManualReady = false;

    System System;
    double servoSpeed = 0;

    boolean fileFound = true;
    Scanner valueReader;

    int gyroAdjust = 0, firstArmAdjust = 0, secondArmAdjust = 0;

    @Override
    public void init(){
        try {
            valueReader = new Scanner(new FileInputStream(AppUtil.ROBOT_DATA_DIR + "position.txt"));
            gyroAdjust = adjustGyro();
            firstArmAdjust = adjustFirstArm();
            secondArmAdjust = adjustSecondArm();
            valueReader.close();
        } catch (FileNotFoundException e) {
            telemetry.addLine("File Not Found");
            fileFound = false;
        }
        m_drive = new DriveBase(hardwareMap, telemetry);
        intake = new Intake(hardwareMap);

        arm = new ftclibArm(hardwareMap, telemetry);

        System = new System(telemetry);

        mode = RobotMode.TELEOP_STARTING;
        telemetry.addData("Status: ", "Initialized");
        telemetry.addData("Robot Mode: ", mode);
    }

    @Override
    public void loop() {
        if(!manualOverrideActive){
            y = -gamepad1.left_stick_y;
            x = gamepad1.left_stick_x * 1.1;
            rx = gamepad1.right_stick_x;

            m_drive.driveByControls(x, y, rx, gyroAdjust);

            mode = RobotMode.TELEOP_RUNNING;
            if(!gamepad2.right_stick_button){
                if(gamepad2.dpad_up) posCode = 0;
                else if (gamepad2.a) posCode = 1;
                else if (gamepad2.b) posCode = 2;
                else if (gamepad2.x) posCode = 3;
                else if (gamepad2.y) posCode = 4;
                arm.moveArmToHeightOfJunction(posCode, firstArmAdjust, secondArmAdjust);
            }

            else if (gamepad2.right_stick_button){
                conesTakenFromStacks = 5 + arm.numOfConesLeft;
                arm.moveArmToHeightOfStacks(posCode, firstArmAdjust, secondArmAdjust);
            }
            telemetry.update();
            //if (gamepad2.dpad_up || gamepad2.dpad_down) arm.ChangeValues(gamepad2.dpad_up, gamepad2.dpad_down);

            //intake
            intake.setServo(-gamepad2.left_stick_y);

            if(gamepad1.start && gamepad1.back && gamepad1.right_bumper && gamepad2.back && gamepad2.start){
                manualOverrideActive = true;
            }
        }

        else if (manualOverrideActive){
            if(!isManualReady){
                arm.disable();
                manualArm = new manualArm(hardwareMap);
                isManualReady = true;
            }
            mode = RobotMode.TELEOP_MANUALOVERRIDE;

            y = -gamepad1.left_stick_y;
            x = gamepad1.left_stick_x * 1.1;
            rx = gamepad1.right_stick_x;

            m_drive.driveByControls(x, y, rx, gyroAdjust);

            manualArm.update(gamepad2.left_stick_y, gamepad2.right_stick_y);

            if(gamepad2.left_bumper) servoSpeed = 1;
            else if(gamepad2.right_bumper) servoSpeed = -1;
            else servoSpeed = 0;
            intake.setServo(servoSpeed);
        }
    }

    private int adjustFirstArm() {
        return valueReader.nextInt();
    }

    private int adjustGyro() {
        return valueReader.nextInt();
    }

    private int adjustSecondArm(){
        return valueReader.nextInt();
    }
}
