package org.firstinspires.ftc.teamcode.Main;

//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.DriveBase;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;

import java.io.FileInputStream;
import java.util.Scanner;


/* This is the main object which will control other objects */
@TeleOp
public class teleOp extends OpMode {
    DriveBase m_drive;
    Intake intake;
    Arm arm;
    //Encoder encoder;

    RobotMode mode;

    double y, x, rx;

    int posCode = 0;

    boolean manualOverrideActive = false;

    @Override
    public void init(){
        mode = RobotMode.TELEOP_STARTING;
        m_drive = new DriveBase(hardwareMap, telemetry);
        intake = new Intake(hardwareMap);
        telemetry.addData("Status: ", "Initialized");
        arm = new Arm(hardwareMap, telemetry);
        //encoder = new Encoder(hardwareMap);

        telemetry.addData("Robot Mode: ", mode);
        try {
            Scanner reader = new Scanner(new FileInputStream(AppUtil.ROBOT_DATA_DIR + "position.txt"));

            // read values left of from auto
            double angle = reader.nextDouble();
            m_drive.last_angle = angle;

        } catch (Exception e) {
            telemetry.addLine(e.getMessage());
            telemetry.update();
        }
    }

    @Override
    public void loop() {
        mode = RobotMode.TELEOP_RUNNING;
        //drive
        y = -gamepad1.left_stick_y;
        x = gamepad1.left_stick_x * 1.1;
        rx = gamepad1.right_stick_x;

        m_drive.driveByControls(x, y, rx);

        //encoder.getCoordinates();

        //telemetry.addData("Last Bot Heading", m_drive.getHeading());;
        //telemetry.addData("x: ", m_drive.position[0]);
        //telemetry.addData("y: ", m_drive.position[1]);

//        telemetry.update();


        //Minor changes done to support Field Oriented

        if (gamepad2.b) {
            posCode = 1;
            // intake.lock();
        }
        else if (gamepad2.x) {
            posCode = 2;
            // intake.unlock();
        }
        else if (gamepad2.y)
            posCode = 3;
        else if (gamepad2.a)
            posCode = 0;

        arm.moveArmToHeightOfJunction(posCode, gamepad2.right_stick_y);
        //arm.MoveManually(gamepad2.right_stick_y, gamepad2.left_stick_y);
        //telemetry.addData("Average: ", arm.getAverageFirst());
        //else if(manualOverrideActive)
        //arm.MoveManually(gamepad2.right_stick_y, gamepad2.left_stick_y);

        //arm.MoveManually(gamepad2.right_stick_y, gamepad2.left_stick_y);
        //telemetry.addData("Height of Arm From Ground", arm.getHeightFromEncoders());


        //telemetry.addData("Last Encoder Pos", Lifter.getLastEncoderPos());
        //telemetry.addData("Currently Encoder Pos", Lifter.getCurrentPos());
//        telemetry.update();

        //intake
        intake.setServo(-gamepad2.left_stick_y);
    }

}
