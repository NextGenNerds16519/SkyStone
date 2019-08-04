package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "DynamicControlUpdated", group = "")
public class DynamicControlUpdated extends OpMode {

    HardwareStraferBasic robot = new HardwareStraferBasic(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    double topLeft = 0;
    double topRight = 0;
    double bottomLeft = 0;
    double bottomRight = 0;

    @Override
    public void init() {
        robot.init(hardwareMap);
    }

    @Override
    public void loop() {
        bottomLeft = gamepad1.left_trigger - gamepad1.right_trigger;
        topLeft = gamepad1.left_trigger - gamepad1.right_trigger;
        topRight = gamepad1.left_trigger - gamepad1.right_trigger;
        bottomRight = gamepad1.left_trigger - gamepad1.right_trigger;
    }

}
