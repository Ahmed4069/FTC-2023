package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    CRServo servo1, servo2;

    public Intake(HardwareMap hardwareMap){
        servo1 = hardwareMap.get(CRServo.class, "intakeLeft");
        servo2 = hardwareMap.get(CRServo.class, "intakeRight");

        servo2.setDirection(DcMotorSimple.Direction.REVERSE);


    }

    public void setServo(double speed){
        servo1.setPower(speed);
        servo2.setPower(speed);
    }

    public void disable(){
        servo1.setPower(0);
        servo2.setPower(0);
    }
}
