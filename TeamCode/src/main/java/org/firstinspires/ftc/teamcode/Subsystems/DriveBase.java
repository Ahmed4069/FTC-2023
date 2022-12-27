// source: https://docs.ftclib.org/ftclib/features/controllers
package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

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
        double botHeading = -gyro.getHeading() + last_angle;

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
}
