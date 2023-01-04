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

    public final int[] requiredAnglesForStacks = {
            350,
            360,
            370,
            380,
            390
    };
    public int numberOfRemainingCones = 5;

    public boolean issueWithPorts = false;
    public boolean issueWithEncoder = false;

    public static int lastIndex = 0;

    public Arm (HardwareMap hardwareMap, Telemetry tele){
        armLift1 = hardwareMap.get(DcMotor.class, "lift1");
        armLift2 = hardwareMap.get(DcMotor.class, "lift2");
        secArmLift1 = hardwareMap.get(DcMotor.class, "secArm1");
        secArmLift2 = hardwareMap.get(DcMotor.class, "secArm2");
        telemetry = tele;

        armLift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armLift1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armLift1.setTargetPosition(0);
        armLift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        armLift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armLift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armLift2.setTargetPosition(0);
        armLift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

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


    public void moveArmToHeightOfJunction(int pos){
            moveFirstArm(0.5, requiredAnglesforClearence[pos][0], telemetry);
            moveSecondArm(0.5, requiredAnglesforClearence[pos][1]);
           lastIndex = pos;
//            telemetry.addData("Index of angle second arm: ", requiredAnglesforClearence[pos][1]);
//            telemetry.addData("Average of First: ", getAverageFirst());
//            telemetry.update();
    }

    public void moveSecondArmToHeightOfStacks(){
        if(numberOfRemainingCones != 0){
            moveSecondArm(0.5, requiredAnglesForStacks[numberOfRemainingCones - 1]);
            numberOfRemainingCones--;
        }
    }

//    private void ManualMovementFirstArm(double speed){
//        //double max = Math.max(Math.abs(speed), 0.2);
//        armLift1.setPower(-speed);
//        armLift2.setPower(-speed);
////        telemetry.addData("angle is ", angle);
////        telemetry.update();
////        armLift1.setTargetPosition(angle);
////        armLift2.setTargetPosition(angle);
////        if (angle > getAverageSecond()){
////            armLift1.setPower(-speed);
////            armLift2.setPower(-speed);
////        }
////        else if (angle < getAverageSecond()){
////            armLift1.setPower(speed);
////            armLift2.setPower(speed);
////        }
////        double differece = angle + getAverageFirst();
////
////        if (differece > 0){
////            armLift1.setPower(-speed);
////            armLift2.setPower(-speed);
////
////            armLift1.setTargetPosition(angle);
////            armLift2.setTargetPosition(angle);
////            differece = angle - getAverageFirst();
////        }
////        else if(differece < 0 ){
////            armLift1.setPower(speed);
////            armLift2.setPower(speed);
////            armLift2.setTargetPosition(-angle);
////            armLift1.setTargetPosition(-angle);
////            differece = angle - getAverageFirst();
////        }
////
////        else if(differece == 0){
////            armLift1.setPower(0);
////            armLift2.setPower(0);
////        }
////        armLift1.setTargetPosition(angle);
////        armLift2.setTargetPosition(angle);
////        telemetry.addData("target pos: ", angle);
////
////        if(angle > getAverageFirst()){
////            armLift1.setPower(speed);
////            armLift2.setPower(speed);
////        }
////        else if (angle < getAverageFirst()){
////            armLift1.setPower(-speed);
////            armLift2.setPower(-speed);
////        }
////
////        if(getAverageFirst() >= angle){
////            armLift1.setPower(0);
////            armLift2.setPower(0);
////        }
//
//    }

    private void moveSecondArm(double speed, int angle){
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

    public double getAverageFirst(){
        return (armLift1.getCurrentPosition() + armLift2.getCurrentPosition()) / 2;
    }
    public double getAverageSecond(){
        return (secArmLift1.getCurrentPosition() + secArmLift2.getCurrentPosition()) / 2;
    }

    public void disable(){
        moveFirstArm(0, 0, telemetry);
        moveSecondArm(0,0);
    }

    public boolean checkFirstArm(){
        if(getAverageFirst() == 0){
            if(armLift2.getPortNumber() == 1 && armLift1.getPortNumber() == 0){
                return true;
            }
            else{
                issueWithPorts = true;
                return false;
            }
        }
        else{
            issueWithEncoder = true;
            return false;
        }
    }

    public boolean checkSecondArm(){
        if(getAverageSecond() == 0){

            if(secArmLift2.getPortNumber() == 3 && secArmLift1.getPortNumber() == 2){
                return true;
            }
            else{
                issueWithPorts = true;
                return false;
            }
        }
        else{
            issueWithEncoder = true;
            return false;
        }
    }

    public void moveFirstArm(double speed, int angle, Telemetry telemetry){
        armLift1.setPower(speed);
        armLift1.setTargetPosition(angle);

        armLift2.setPower(-speed);
        armLift2.setTargetPosition(-angle);

        telemetry.addData("In: ", "Moving First Arm");
    }

    public void updateHeight(){

    }


}
