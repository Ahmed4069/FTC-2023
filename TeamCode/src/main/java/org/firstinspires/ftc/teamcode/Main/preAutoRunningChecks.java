package org.firstinspires.ftc.teamcode.Main;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Main.Robot.Robot;
import org.firstinspires.ftc.teamcode.Vision.SignalClassifier;

public class preAutoRunningChecks {
    Robot robot;
    Telemetry telemetry;
    SignalClassifier classifier;
    public boolean driveMotors, gyroParams, armEncoder1, armEncoder2, intakePlugs, IsClassificationLoaded;
    public boolean robotIsReady = false;

    public preAutoRunningChecks(Robot robot, Telemetry telemetry, SignalClassifier classifier){
        this.robot = robot;
        this.telemetry = telemetry;
        this.classifier = classifier;
    }

    public double errorWithLeftDrive, errorWithRightDrive, errorWithArm1, errorWithArm2;

    public void checkForErrors(){
        gyroParams = robot.gyro.checkParams();
        driveMotors = robot.driveBase.driveBaseCheck();
        armEncoder1 = robot.arm.checkFirstArm();
        armEncoder2 = robot.arm.checkSecondArm();
        intakePlugs = robot.intake.checkServo();
        IsClassificationLoaded = SignalClassifier.isClassificationLoaded;
    }

    public boolean isRobotPrepared(){
        if(gyroParams && intakePlugs && driveMotors && armEncoder1 && armEncoder2 && IsClassificationLoaded){
            robotIsReady = true;
            return true;
        }
        else{
            fixErrors();
            return false;
        }
    }

    public void fixErrors(){
        if (!gyroParams){
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
            robot.gyro.imu.initialize(parameters);
        }
        if (!driveMotors){
            if(!robot.driveBase.encodersClear){
                errorWithLeftDrive = robot.driveBase.getLeftEncoderValue();
                errorWithRightDrive = robot.driveBase.getRightEncoderValue();
            }
            if(!robot.driveBase.portsClear){
                telemetry.log().add("Drive Base Configuration Error Found. Please Fix");
            }
        }
        if(!armEncoder1){
            if(!robot.arm.issueWithPorts){
                telemetry.log().add("Arm Motor 1 Configuration Error Found. Please Fix");
            }
            if(!robot.arm.issueWithEncoder){
                errorWithArm1 = robot.arm.getAverageFirst();
                errorWithArm2 = robot.arm.getAverageSecond();
            }
        }
        if(!armEncoder2){
            if(!robot.arm.issueWithPorts){
                telemetry.log().add("Arm Motor 2 Configuration Error Found. Please Fix");
            }
        }
        if(!intakePlugs){
            if(!robot.intake.issueWithLock){
                robot.intake.unlock();
            }
            if(!robot.intake.issueWithPorts){
                telemetry.log().add("Servo Configuration Error Found. Please Fix");
            }
        }
        if(!IsClassificationLoaded){
            telemetry.log().add("Classification Not Loaded. Resetting classifier. This WILL cause a delay");
            classifier.resetClassification();
        }
        robotIsReady = true;
    }
}

