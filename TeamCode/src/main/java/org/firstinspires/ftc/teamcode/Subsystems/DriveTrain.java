package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DriveTrain {
    private Motor motorRF, motorLF, motorRB, motorLB;
    private Gyro gyro;
    LED_Controller led_controller;

    MecanumDrive driveController;

    public DriveTrain(HardwareMap hardwareMap, Gyro gyro, LED_Controller led_controller){
        this.led_controller = led_controller;
        motorRF = new Motor(hardwareMap, "motorRF");
        motorRB = new Motor(hardwareMap, "motorRB");
        motorLF = new Motor(hardwareMap, "motorLF");
        motorLB = new Motor(hardwareMap, "motorLB");

        this.gyro = gyro;

        driveController = new MecanumDrive(motorLF, motorRF, motorLB, motorRB);
    }

    public void setMotorPowers(double rfPower, double lfPower, double lbPower, double rbPower){
        motorLB.set(lbPower);
        motorLF.set(lfPower);
        motorRB.set(rbPower);
        motorRF.set(rfPower);
    }

    public void update(double strafe, double forwardSpeed, double turn, double heading){
        driveController.driveFieldCentric(strafe, forwardSpeed, turn, heading);
    }
}
