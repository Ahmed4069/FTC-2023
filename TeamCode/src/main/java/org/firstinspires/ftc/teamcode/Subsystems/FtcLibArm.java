package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ftclibArm {
    Motor secArmLift1, secArmLift2;
    Motor armLift1, armLift2;
    Telemetry telemetry;
    boolean atPos = false, atPos1 = false;

    private int[][] requiredAnglesforClearence = {
            {0, 0},        // front
            {0, 1600},      // ground
            {0, 450},        // low
            {3500, 1600},     // medium
            {3500, 3350} ,    // high
            //{0, 100} //upside down intake
            //stacks
            {0, 1200},
            {0, 1430},
            {0, 1480},
            {0, 1840},
            {0, 2000}
    };

    private int[] stackValues = {1244, 1430, 1480, 1840, 2000};

    double kP = 0.0025, S_kP = 0.002;

    int lastPos = 0;
    public int numOfConesLeft = 0;
    int initialPos = 0, sec_initialPos = 0;

    public ftclibArm(HardwareMap hardwareMap, Telemetry tele) {
        secArmLift1 = new Motor(hardwareMap, "secArm1");
        secArmLift2 = new Motor(hardwareMap, "secArm2");
        telemetry = tele;
        // lower arm setup
        armLift1 = new Motor(hardwareMap, "lift1");
        armLift2 = new Motor(hardwareMap, "lift2");
        telemetry = tele;

        armLift1.resetEncoder();
        armLift2.resetEncoder();

        initialPos = armLift1.getCurrentPosition();

        armLift1.setRunMode(Motor.RunMode.PositionControl);
        armLift2.setRunMode(Motor.RunMode.PositionControl);

        armLift1.setInverted(true);

        armLift1.setPositionCoefficient(kP);
        armLift2.setPositionCoefficient(kP);

        armLift1.setPositionTolerance(100);
        armLift2.setPositionTolerance(100);

        armLift1.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        armLift2.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        armLift1.setTargetPosition(0);
        armLift2.setTargetPosition(0);


        // upper arm setup
        secArmLift1.resetEncoder();
        secArmLift2.resetEncoder();

        initialPos = armLift2.getCurrentPosition();

        secArmLift1.setRunMode(Motor.RunMode.PositionControl);
        secArmLift2.setRunMode(Motor.RunMode.PositionControl);

        secArmLift1.setInverted(true);

        secArmLift1.setPositionCoefficient(S_kP);
        secArmLift2.setPositionCoefficient(S_kP);

        secArmLift1.setPositionTolerance(25);
        secArmLift2.setPositionTolerance(25);

        secArmLift1.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        secArmLift2.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        secArmLift1.setTargetPosition(0);
        secArmLift2.setTargetPosition(0);

        sec_initialPos = secArmLift2.getCurrentPosition();

        armLift1.resetEncoder();
        armLift2.resetEncoder();
        secArmLift1.resetEncoder();
        secArmLift2.resetEncoder();
    }


    public void moveArmToHeightOfJunction(int pos, int armAdjust1, int armAdjust2) {

//        if(lastPos < pos){
//            if (!armLift1.atTargetPosition()) {
//                moveFirstArm(-0.55, requiredAnglesforClearence[pos][0], telemetry);
//                telemetry.addLine("condition 1");
//            } else {
//                moveSecondArm(-0.75, requiredAnglesforClearence[pos][1]);
//                armLift1.set(0);
//                armLift2.set(0);
//                telemetry.addLine("condition 2");
//            }
//        }
//        else if(lastPos > pos){
//            telemetry.addLine("going down");
//            if (!(Math.abs(getAverageSecond() - requiredAnglesforClearence[pos][1]) < 5)) {
//                moveSecondArm(-0.75, requiredAnglesforClearence[pos][1]);
//                telemetry.addLine("condition 3");
//            } else {
//                moveFirstArm(-0.55, requiredAnglesforClearence[pos][0], telemetry);
//                telemetry.addLine("condition 4");
//            }
//        }
//        telemetry.update();
        armLift1.setTargetPosition(requiredAnglesforClearence[pos][0] - armAdjust1);
        armLift2.setTargetPosition(requiredAnglesforClearence[pos][0] - armAdjust1);
        secArmLift1.setTargetPosition(requiredAnglesforClearence[pos][1] -armAdjust2);
        secArmLift2.setTargetPosition(requiredAnglesforClearence[pos][1] - armAdjust2);

        if (armLift2.getCurrentPosition() < (requiredAnglesforClearence[pos][0] - armAdjust1)) {
            telemetry.addLine("going up");
            // move up: first arm moves first
            atPos1 = false;
            if (!armLift2.atTargetPosition() && !atPos1) {
                moveFirstArm(-0.90, (requiredAnglesforClearence[pos][0] - armAdjust1), telemetry);
                stopSecondArm();
                telemetry.addLine("condition 1");
            } else {
                moveSecondArm(-0.70, requiredAnglesforClearence[pos][1] - armAdjust2, telemetry);
                stopFirstArm();
                telemetry.addLine("condition 2");
            }
        } else {
            telemetry.addLine("going down");
            atPos = false;
            if (!secArmLift2.atTargetPosition() && !atPos
                    && Math.abs(secArmLift2.getCurrentPosition() - (requiredAnglesforClearence[pos][1] - armAdjust2)) > 170) {
                moveSecondArm(-0.70, requiredAnglesforClearence[pos][1] - armAdjust2, telemetry);
                stopFirstArm();
                telemetry.addLine("condition 3");
            } else {
                moveFirstArm(-0.90, (requiredAnglesforClearence[pos][0] - armAdjust1), telemetry);
                stopSecondArm();
                telemetry.addLine("condition 4");
            }
        }

//        if(pos > lastPos){
//            moveFirstArm(-0.55, requiredAnglesforClearence[pos][0] - initialPos, telemetry);
//            moveSecondArm(-0.55, requiredAnglesforClearence[pos][1]);
//        }
//        else if(pos < lastPos){
//            moveSecondArm(-0.55, requiredAnglesforClearence[pos][1] - initialPos);
//            moveFirstArm(-0.55, requiredAnglesforClearence[pos][0], telemetry);
//        }

        telemetry.update();

        lastPos = pos;
    }

    public void moveArmToHeightOfStacks(int pos, int first, int second) {

        if(!(pos <= 5 && pos >= 0)){
            return;
        }

//        if(lastPos < pos){
//            if (!armLift1.atTargetPosition()) {
//                moveFirstArm(-0.55, requiredAnglesforClearence[pos][0], telemetry);
//                telemetry.addLine("condition 1");
//            } else {
//                moveSecondArm(-0.75, requiredAnglesforClearence[pos][1]);
//                armLift1.set(0);
//                armLift2.set(0);
//                telemetry.addLine("condition 2");
//            }
//        }
//        else if(lastPos > pos){
//            telemetry.addLine("going down");
//            if (!(Math.abs(getAverageSecond() - requiredAnglesforClearence[pos][1]) < 5)) {
//                moveSecondArm(-0.75, requiredAnglesforClearence[pos][1]);
//                telemetry.addLine("condition 3");
//            } else {
//                moveFirstArm(-0.55, requiredAnglesforClearence[pos][0], telemetry);
//                telemetry.addLine("condition 4");
//            }
//        }
//        telemetry.update();
        armLift1.setTargetPosition(stackValues[pos] - first);
        armLift2.setTargetPosition(stackValues[pos] - first);
        secArmLift1.setTargetPosition(stackValues[pos] - second);
        secArmLift2.setTargetPosition(stackValues[pos] - second);

        if (armLift2.getCurrentPosition() < (stackValues[pos] - first)) {
            telemetry.addLine("going up");
            // move up: first arm moves first
            atPos1 = false;
            if (!armLift2.atTargetPosition() && !atPos1) {
                moveFirstArm(-0.55, 0, telemetry);
                stopSecondArm();
                telemetry.addLine("condition 1");
            } else {
                moveSecondArm(-0.55, stackValues[pos] - second, telemetry);
                stopFirstArm();
                telemetry.addLine("condition 2");
            }
        } else {
            telemetry.addLine("going down");
            atPos = false;
            if (!secArmLift2.atTargetPosition() && !atPos
                    && Math.abs(secArmLift2.getCurrentPosition() - (stackValues[pos] - second)) > 100) {
                moveSecondArm(-0.55, stackValues[pos] - second, telemetry);
                stopFirstArm();
                telemetry.addLine("condition 3");
            } else {
                moveFirstArm(-0.55, 0, telemetry);
                stopSecondArm();
                telemetry.addLine("condition 4");
            }
        }
        if((secArmLift2.getCurrentPosition() - (stackValues[pos] - second)) > 100 && armLift2.getCurrentPosition() == 0){
            numOfConesLeft++;
            telemetry.update();
            lastPos = pos;
            return;
        }


//        if(pos > lastPos){
//            moveFirstArm(-0.55, requiredAnglesforClearence[pos][0] - initialPos, telemetry);
//            moveSecondArm(-0.55, requiredAnglesforClearence[pos][1]);
//        }
//        else if(pos < lastPos){
//            moveSecondArm(-0.55, requiredAnglesforClearence[pos][1] - initialPos);
//            moveFirstArm(-0.55, requiredAnglesforClearence[pos][0], telemetry);
//        }
    }

    public boolean atPosition() {
        return armLift1.atTargetPosition() && secArmLift1.atTargetPosition() || (atPos && atPos1);
    }

    public double diff() {
        telemetry.addData("lastPos", lastPos);
        return Math.abs(secArmLift1.getCurrentPosition() + requiredAnglesforClearence[lastPos][1]);
    }

    private void moveFirstArm(double speed, int angle, Telemetry telemetry) {
        armLift1.setTargetPosition(angle);
        armLift2.setTargetPosition(angle);


        if (!armLift2.atTargetPosition()) {
            telemetry.addLine("not stopped");
            telemetry.addData("target", angle);
            telemetry.addData("current", armLift1.getCurrentPosition());
            armLift1.set(speed);
            armLift2.set(speed);

        } else {
            telemetry.addLine("stopped");
            armLift1.stopMotor();
            armLift2.stopMotor();
            atPos1 = true;
            return;
        }

        telemetry.update();
    }

    public void stopFirstArm() {
        armLift1.stopMotor();
        armLift2.stopMotor();
    }
    private void moveSecondArm (double speed, int angle, Telemetry telemetry){
        secArmLift1.setTargetPosition(angle);
        secArmLift2.setTargetPosition(angle);


        if (!secArmLift2.atTargetPosition()) {
            telemetry.addLine("not stopped 2");
            telemetry.addData("target 2 ", angle);
            telemetry.addData("current 2", secArmLift2.getCurrentPosition());
            secArmLift1.set(-speed);
            secArmLift2.set(-speed);

        } else if (secArmLift2.atTargetPosition()) {
            telemetry.addLine("stopped2 ");
            secArmLift1.stopMotor();
            secArmLift2.stopMotor();
            atPos = true;
            return;
        }

        telemetry.update();
    }

    public void stopSecondArm() {
        secArmLift1.stopMotor();
        secArmLift2.stopMotor();
    }

    public double getAverageFirst() {return armLift2.getCurrentPosition();}

    public double getAverageSecond () {return secArmLift2.getCurrentPosition();}

    public void disable () {
        moveFirstArm(0, 0, telemetry);
        moveSecondArm(0, 0, telemetry);
    }

    public void ChangeValues(boolean up, boolean down){
        if(up) requiredAnglesforClearence[1][1] += 1;
        else if(down) requiredAnglesforClearence[1][1] -= 1;
        else return;
    }
}
