package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Arm {
    DcMotor secArmLift1, secArmLift2;
    Motor armLift1, armLift2;
    Telemetry telemetry;

    public final int[][] requiredAnglesforClearence = {
            {0, 0},        // front
            {0, 146},      // ground
            {0, 50},        // low
            {2300, 155},     // medium
            {2300, 230}     // high
    };

    public final int[][] requiredAnglesForStacks = {
            {198, 170},
            {13, 150},
            {0, 155},
            {0, 160},
            {0, 175}
    };

    double kP = 0.0025;

    int lastPos = 0;
    public int numOfConesLeft = 0;

    public Arm(HardwareMap hardwareMap, Telemetry tele) {
        secArmLift1 = hardwareMap.get(DcMotor.class, "secArm1");
        secArmLift2 = hardwareMap.get(DcMotor.class, "secArm2");
        telemetry = tele;
        // lower arm setup
        armLift1 = new Motor(hardwareMap, "lift1");
        armLift2 = new Motor(hardwareMap, "lift2");
        telemetry = tele;

        armLift1.resetEncoder();
        armLift2.resetEncoder();

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
    }


    public void moveArmToHeightOfJunction(int pos) {
        telemetry.addData("first arm position", armLift1.getCurrentPosition());
        telemetry.addData("second arm position", getAverageSecond());

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
        armLift1.setTargetPosition(requiredAnglesforClearence[pos][0]);
        armLift2.setTargetPosition(requiredAnglesforClearence[pos][0]);
        secArmLift1.setTargetPosition(-requiredAnglesforClearence[pos][1]);
        secArmLift2.setTargetPosition(-requiredAnglesforClearence[pos][1]);

        if (armLift1.getCurrentPosition() < requiredAnglesforClearence[pos][0]) {
            telemetry.addLine("going up");
            // move up: first arm moves first
            if (!armLift1.atTargetPosition()) {
                moveFirstArm(-0.55, requiredAnglesforClearence[pos][0], telemetry);
                stopSecondArm();
                telemetry.addLine("condition 1");
            } else {
                moveSecondArm(-0.75, requiredAnglesforClearence[pos][1]);
                stopFirstArm();
                telemetry.addLine("condition 2");
            }
        } else {
            telemetry.addLine("going down");
            if (!(Math.abs(getAverageSecond() + requiredAnglesforClearence[pos][1]) < 6)) {
                moveSecondArm(-0.75, requiredAnglesforClearence[pos][1]);
                stopFirstArm();
                telemetry.addLine("condition 3");
            } else {
                moveFirstArm(-0.55, requiredAnglesforClearence[pos][0], telemetry);
                stopSecondArm();
                telemetry.addLine("condition 4");
            }
        }

        telemetry.update();

        lastPos = pos;
    }

    public void moveArmToHeightOfStacks(){
        telemetry.addData("first arm position", armLift1.getCurrentPosition());
        telemetry.addData("second arm position", getAverageSecond());

        if(numOfConesLeft > 5){
            numOfConesLeft = 0;
        }

        if (requiredAnglesforClearence[lastPos][0] > requiredAnglesForStacks[numOfConesLeft][0]) {
            if (!armLift1.atTargetPosition()) {
                moveFirstArm(-0.55, requiredAnglesForStacks[numOfConesLeft][0], telemetry);
                // telemetry.addLine("condition 1");
            } else {
                moveSecondArm(-0.75, requiredAnglesForStacks[numOfConesLeft][1]);
                armLift1.set(0);
                armLift2.set(0);
                //telemetry.addLine("condition 2");
            }
        } else if (requiredAnglesforClearence[lastPos][0] < requiredAnglesForStacks[numOfConesLeft][0]) {
            if (!(Math.abs(getAverageSecond() - requiredAnglesForStacks[numOfConesLeft][1]) < 5)) {
                moveSecondArm(-0.75, requiredAnglesforClearence[numOfConesLeft][1]);
                //telemetry.addLine("condition 3");
            } else {
                moveFirstArm(-0.55, requiredAnglesForStacks[numOfConesLeft][0], telemetry);
                //telemetry.addLine("condition 4");
            }
        }


    }

    public boolean atPosition() {
        return armLift1.atTargetPosition() && (Math.abs(getAverageSecond() + requiredAnglesforClearence[lastPos][1]) < 6);
    }

    public double diff() {
        telemetry.addData("lastPos", lastPos);
        return Math.abs(getAverageSecond() + requiredAnglesforClearence[lastPos][1]);
    }

    private void moveFirstArm(double speed, int angle, Telemetry telemetry) {
        armLift1.setTargetPosition(angle);
        armLift2.setTargetPosition(angle);

        if (!armLift1.atTargetPosition()) {
            telemetry.addLine("not stopped");
            telemetry.addData("target", angle);
            telemetry.addData("current", armLift1.getCurrentPosition());
            armLift1.set(speed);
            armLift2.set(speed);
        } else {
            telemetry.addLine("stopped");
            stopFirstArm();
        }

        telemetry.update();
    }

    public void stopFirstArm() {
        armLift1.stopMotor();
        armLift2.stopMotor();
    }

    private void moveSecondArm ( double speed, int angle){
        if (angle > getAverageSecond()) {
            secArmLift1.setPower(-speed);
            secArmLift2.setPower(-speed);
        } else if (angle < getAverageSecond()) {
            secArmLift1.setPower(speed);
            secArmLift2.setPower(speed);
        } else {
            secArmLift1.setPower(0);
            secArmLift2.setPower(0);
        }
    }

    public void stopSecondArm() {
        secArmLift1.setPower(0);
        secArmLift2.setPower(0);
    }

    public double getAverageSecond () {
        return (secArmLift1.getCurrentPosition() + secArmLift2.getCurrentPosition()) / 2;
    }

    public void disable () {
        moveFirstArm(0, 0, telemetry);
        moveSecondArm(0, 0);
    }

}