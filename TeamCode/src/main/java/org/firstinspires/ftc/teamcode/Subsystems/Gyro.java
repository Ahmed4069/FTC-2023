package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class Gyro {
    public BNO055IMU imu;
    double lastHeading = 0;

    public Gyro(HardwareMap hardwareMap){
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);
    }

    public double getHeading(){
        return imu.getAngularOrientation().firstAngle;
    }

    public boolean checkParams(){
        if(imu.isGyroCalibrated()){
            return true;
        }
        else{
            return false;
        }
    }
}
