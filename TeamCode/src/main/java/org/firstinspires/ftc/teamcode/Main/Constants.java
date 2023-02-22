package org.firstinspires.ftc.teamcode.Main;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;

public class Constants {
    public static File classification_model = new File(AppUtil.ROBOT_DATA_DIR, "models/classification/signal_classifier2.tflite");
    public static File ValueSaver = new File("TeamCode/java/or.firstinspires.ftc.teamcode/Autonomous/Auto/values.txt");


    public static final byte motorRfPort = 0;
    public static final byte motorLfPort = 1;
    public static final byte motorRbPort = 2;
    public static final byte motorLbPort = 3;

    public static final double encoderToFeetConstant = 0.000054784907761;
    public static final double encoderToInchesConstant = 0.00065741889313;
}
