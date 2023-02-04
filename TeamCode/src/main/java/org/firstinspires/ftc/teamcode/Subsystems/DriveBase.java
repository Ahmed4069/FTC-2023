// source: https://docs.ftclib.org/ftclib/features/controllers
package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Main.Constants;

public class DriveBase {
    // define the hardware
    private DcMotorEx motorRF, motorRB, motorLF, motorLB;

    private DcMotorEx encL, encR;
    private Gyro gyro;
    public double last_angle = 0;

    // define variables for odometry
    private double lastForwardPos = 0, lastSidewaysPos = 0;
    public double[] last_position = {0, 0};

    Telemetry telemetry;

    private Constants constants;

    public boolean encodersClear = false;
    public boolean portsClear = false;

    double lastLeft = 0, lastRight = 0, lastHeading = 0;
    Pose2d currentPose;

    public DriveBase(HardwareMap hardwareMap, Telemetry tele) {
        telemetry = tele;

        motorRF = hardwareMap.get(DcMotorEx.class, "motorRF");
        motorLF = hardwareMap.get(DcMotorEx.class, "motorLF");
        motorRB = hardwareMap.get(DcMotorEx.class, "motorRB");
        motorLB = hardwareMap.get(DcMotorEx.class, "motorLB");

        motorLF.setDirection(DcMotor.Direction.REVERSE);
        motorLB.setDirection(DcMotor.Direction.REVERSE);

        motorRF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorRF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorLF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorLB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        gyro = new Gyro(hardwareMap);
    }

    public void setMotorPowers(double p_lf, double p_lb, double p_rf, double p_rb) {
        motorLF.setPower(p_lf);
        motorLB.setPower(p_lb);
        motorRF.setPower(p_rf);
        motorRB.setPower(p_rb);
    }

    /** a function to drive by controls, field oriented */
    public void driveByControls(double x, double y, double rx) {
        double botHeading = -gyro.getHeading();

        double rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
        double rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]

        double powerLF = (rotY + rotX + rx) / denominator;
        double powerLB = (rotY - rotX + rx) / denominator;
        double powerRF = (rotY - rotX - rx) / denominator;
        double powerRB = (rotY + rotX - rx) / denominator;

        setMotorPowers(powerLF, powerLB, powerRF, powerRB);
    }

    public void tankDrive(double left, double right) {
        setMotorPowers(left, left, right, right);
    }

    public void driveByTrigger(double speed){
        double leftPower = speed + (getHeading() * 0.2);
        double rightPower = speed - (getHeading() * 0.2);

        setMotorPowers(leftPower, leftPower, rightPower, rightPower);
    }

    public void stop() {
        setMotorPowers(0, 0, 0, 0);
    }

    public double getHeading() {
        return gyro.getHeading();
    }

    public void print() {
        telemetry.addData("left encoder", getLeftEncoderValue());
        telemetry.addData("right encoder", getRightEncoderValue());
    }

    public void disable(){
        stop();
    }

    public int getLeftEncoderValue(){
        return motorLF.getCurrentPosition();
    }

    public int getRightEncoderValue(){
        return motorRF.getCurrentPosition();
    }

    public boolean driveBaseCheck(){
        if(getLeftEncoderValue() == 0 && getRightEncoderValue() == 0){
            encodersClear = true;
            if(motorRF.getPortNumber() == Constants.motorRfPort && motorLF.getPortNumber() == Constants.motorLfPort && motorRB.getPortNumber() == Constants.motorRbPort && motorLB.getPortNumber() == Constants.motorLbPort){
                portsClear = true;
            }
        }

        if (portsClear && encodersClear){
            return true;
        }
        else{
            return false;
        }
    }

    public void resetPositions(){
        motorLF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        telemetry.addLine("Restarting...");

        lastLeft = 0;
        lastRight = 0;
        currentPose = new Pose2d(new Translation2d(0, 0), new Rotation2d(Math.toRadians(gyro.getHeading())));
    }

    public void updatePosition(){
        double currentLeft = getLeftEncoderValue();
        double currentRight = getRightEncoderValue();

        double deltaLeft = currentLeft - lastLeft;
        double deltaRight = currentRight - lastRight;

        double heading = gyro.getHeading();
        double deltaHeading = heading - lastHeading;

        double distance = (deltaLeft + deltaRight) / 2.0;

        Translation2d translation = new Translation2d(distance * Math.cos(Math.toRadians(deltaHeading)), distance * Math.sin(Math.toRadians(deltaHeading)));
        Rotation2d rotation = new Rotation2d(Math.toRadians(deltaHeading));

        currentPose = currentPose.transformBy(new Transform2d(translation, rotation));

        lastRight = getRightEncoderValue();
        lastLeft = getLeftEncoderValue();
        lastHeading = gyro.getHeading();
    }

    public Pose2d getPose(){
        return currentPose;
    }
}
