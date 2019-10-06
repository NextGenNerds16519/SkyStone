package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "TariniTeleOp", group = "TeamPractice")
public class TariniTeleOp extends OpMode {

    HardwareStrafer robot = new HardwareStrafer(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


    @Override
    public void init() {
        robot.init(hardwareMap);
    }

    @Override
    public void loop() {
        if (gamepad1.left_bumper && gamepad1.right_bumper){
               // robot.moveForward(34);
        }

    }
}
