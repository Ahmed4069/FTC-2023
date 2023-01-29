package org.firstinspires.ftc.teamcode.Main;

//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.DriveBase;
import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.Subsystems.FtcLibArm;
import org.firstinspires.ftc.teamcode.Subsystems.Gyro;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.LED_Controller;


/* This is the main object which will control other objects */
@TeleOp
public class FtcLibTeleOp extends OpMode {
    DriveTrain mechDrive;
    Intake intake;
    FtcLibArm arm;
    Gyro gyro;
    GamepadEx driverOne, driverTwo;
    RobotMode mode;
    LED_Controller leds;
    int posCode = 0;

    boolean manualOverrideActive = false;

    System System;

    @Override
    public void init(){
        leds = new LED_Controller(hardwareMap);
        mode = RobotMode.TELEOP_STARTING;
        intake = new Intake(hardwareMap);
        telemetry.addData("Status: ", "Initialized");
        arm = new FtcLibArm(hardwareMap, telemetry, leds);
        gyro = new Gyro(hardwareMap);

        mechDrive = new DriveTrain(hardwareMap, gyro, leds);

        telemetry.addData("Robot Mode: ", mode);

        System = new System(telemetry);

        driverOne = new GamepadEx(gamepad1);
        driverTwo = new GamepadEx(gamepad2);
    }

    @Override
    public void loop() {
        mode = RobotMode.TELEOP_RUNNING;

        mechDrive.update(driverOne.getLeftX(), driverOne.getLeftY(), driverOne.getRightX(), gyro.getHeading());

        if(driverTwo.getButton(GamepadKeys.Button.DPAD_UP)) posCode = 0;
        else if (driverTwo.getButton(GamepadKeys.Button.A)) posCode = 1;
        else if (driverTwo.getButton(GamepadKeys.Button.B)) posCode = 2;
        else if (driverTwo.getButton(GamepadKeys.Button.X)) posCode = 3;
        else if (driverTwo.getButton(GamepadKeys.Button.Y)) posCode = 4;

        arm.moveArmToHeightOfJunction(posCode);
        telemetry.update();

        if (driverTwo.getButton(GamepadKeys.Button.RIGHT_BUMPER)){
            arm.moveArmToHeightOfStacks();
            System.printDouble("Num of remaining cones: ", arm.numOfConesLeft);
        }

        intake.setServo(driverTwo.getLeftY());
    }

}
