package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LED_Controller {
    RevBlinkinLedDriver blinkin;
    public LED_Controller(HardwareMap hardwareMap){
        blinkin = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
    }
    public void update(RevBlinkinLedDriver.BlinkinPattern pattern){blinkin.setPattern(pattern);}
}
