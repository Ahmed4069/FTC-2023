package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake {
    CRServo servo1, servo2;
    Servo lock;

    public Intake(HardwareMap hardwareMap){
        servo1 = hardwareMap.get(CRServo.class, "intakeLeft");
        servo2 = hardwareMap.get(CRServo.class, "intakeRight");
        servo2.setDirection(DcMotorSimple.Direction.REVERSE);

//        lock = hardwareMap.get(Servo.class, "lock");
//        lock.setPosition(1);
    }

    public void setServo(double speed){
        servo1.setPower(speed);
        servo2.setPower(speed);
    }

    public void disable(){
        servo1.setPower(0);
        servo2.setPower(0);
    }

//    public void lock(){
//        lock.setPosition(1);
//    }
//    public void unlock(){
//        lock.setPosition(0);
//    }
}
