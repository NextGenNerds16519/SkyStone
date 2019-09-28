package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "CalebTeleOp", group = "Final")
public class CalebTeleOp extends OpMode {

    HardwareStrafer robot = new HardwareStrafer(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    @Override
    public void init(){
        robot.init(hardwareMap);

    }

    @Override
    public void loop(){

        float[] leftStick = {gamepad1.left_stick_x, gamepad1.left_stick_y};

        robot.totalControl(gamepad1.right_trigger, gamepad1.left_trigger, leftStick, gamepad1.right_bumper, gamepad1.left_bumper);

        telemetry.addData("Top Left Pow", robot.topLeftDrive.getPower());
        telemetry.addData("Bottom Left Pow", robot.bottomLeftDrive.getPower());
        telemetry.addData("Top Right Pow", robot.topRightDrive.getPower());
        telemetry.addData("Bottom Right Pow", robot.bottomRightDrive.getPower());
        telemetry.addData("Heading", robot.angle);
        telemetry.update();

    }
}
