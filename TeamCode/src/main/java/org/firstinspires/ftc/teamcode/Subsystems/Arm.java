package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Arm{
    DcMotor armLift1, armLift2, secArmLift1, secArmLift2;
    Servo lock;

    Telemetry telemetry;

    private final double highJunctionHeight = 33.5 ;
    private final double MediumJunctionHeight = 23.5;
    private final double LowJunctionHeight =  13.5;
    private final double groundJunctionheight = 1;

    private final double fullArmLength = 20.5;
    private final double firstArmLength = 12.5;
    private final double secondArmLength = 8;
    private final double armHeight = 15.5;

    public final int[][] requiredAnglesforClearence = {
            {0 , 0},        // front
            {0, 350},      // ground
            {500, 350},     // medium
            {500, 525}     // high
    };


    public Arm (HardwareMap hardwareMap, Telemetry tele){
        armLift1 = hardwareMap.get(DcMotor.class, "lift1");
        armLift2 = hardwareMap.get(DcMotor.class, "lift2");
        secArmLift1 = hardwareMap.get(DcMotor.class, "secArm1");
        secArmLift2 = hardwareMap.get(DcMotor.class, "secArm2");
        telemetry = tele;

        armLift1.setDirection(DcMotorSimple.Direction.REVERSE);
        armLift1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        armLift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        secArmLift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        secArmLift1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        secArmLift1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        secArmLift1.setTargetPosition(0);
       secArmLift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        secArmLift1.setDirection(DcMotorSimple.Direction.REVERSE);

        secArmLift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        secArmLift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        secArmLift2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        secArmLift2.setTargetPosition(0);
        secArmLift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

//        lock = hardwareMap.get(Servo.class, "lock");
//        lock.setPosition(1);
    }


    public void moveArmToHeightOfJunction(int pos, double FistSpeed){
            moveFirstArm(FistSpeed);
            moveSecondArm(0.5, requiredAnglesforClearence[pos][1]);

            telemetry.addData("Index of angle second arm: ", requiredAnglesforClearence[pos][1]);
            telemetry.addData("Average of First: ", getAverageFirst());
            telemetry.update();
    }

    public void moveFirstArm(double speed){
        //double max = Math.max(Math.abs(speed), 0.2);
        armLift1.setPower(-speed);
        armLift2.setPower(-speed);
//        telemetry.addData("angle is ", angle);
//        telemetry.update();
//        armLift1.setTargetPosition(angle);
//        armLift2.setTargetPosition(angle);
//        if (angle > getAverageSecond()){
//            armLift1.setPower(-speed);
//            armLift2.setPower(-speed);
//        }
//        else if (angle < getAverageSecond()){
//            armLift1.setPower(speed);
//            armLift2.setPower(speed);
//        }
//        double differece = angle + getAverageFirst();
//
//        if (differece > 0){
//            armLift1.setPower(-speed);
//            armLift2.setPower(-speed);
//
//            armLift1.setTargetPosition(angle);
//            armLift2.setTargetPosition(angle);
//            differece = angle - getAverageFirst();
//        }
//        else if(differece < 0 ){
//            armLift1.setPower(speed);
//            armLift2.setPower(speed);
//            armLift2.setTargetPosition(-angle);
//            armLift1.setTargetPosition(-angle);
//            differece = angle - getAverageFirst();
//        }
//
//        else if(differece == 0){
//            armLift1.setPower(0);
//            armLift2.setPower(0);
//        }
//        armLift1.setTargetPosition(angle);
//        armLift2.setTargetPosition(angle);
//        telemetry.addData("target pos: ", angle);
//
//        if(angle > getAverageFirst()){
//            armLift1.setPower(speed);
//            armLift2.setPower(speed);
//        }
//        else if (angle < getAverageFirst()){
//            armLift1.setPower(-speed);
//            armLift2.setPower(-speed);
//        }
//
//        if(getAverageFirst() >= angle){
//            armLift1.setPower(0);
//            armLift2.setPower(0);
//        }

    }

    public void moveSecondArm(double speed, int angle){
        secArmLift1.setTargetPosition(angle);
        secArmLift2.setTargetPosition(angle);
        if (angle > getAverageSecond()){
            secArmLift1.setPower(-speed);
            secArmLift2.setPower(-speed);
        }
        else if (angle < getAverageSecond()){
            secArmLift1.setPower(speed);
            secArmLift2.setPower(speed);
        }
    }

    public void MoveManually(double firstSpeed, double SecondSpeed){
        armLift1.setPower(firstSpeed);
        armLift2.setPower(firstSpeed);
        secArmLift2.setPower(SecondSpeed);
        secArmLift1.setPower(SecondSpeed);
    }

    public double getAverageFirst(){
        return (armLift1.getCurrentPosition() + armLift2.getCurrentPosition()) / 2;
    }
    public double getAverageSecond(){
        return (secArmLift1.getCurrentPosition() + secArmLift2.getCurrentPosition()) / 2;
    }

    public void disable(){
        moveFirstArm(0);
        moveSecondArm(0,0);
    }

}
