package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "StraferTeleOp", group = "Final")
public class StraferTeleOp extends OpMode {

    HardwareStrafer robot = new HardwareStrafer(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    @Override
    public void init() {

        robot.init(hardwareMap);

    }

    @Override
    public void loop() {

        if (gamepad1.left_bumper && gamepad1.left_trigger == 0 && gamepad1.right_trigger == 0) {

            robot.strafeLeft(0.75);

        } else if (gamepad1.right_bumper && gamepad1.left_trigger == 0 && gamepad1.right_trigger == 0) {

            robot.strafeRight(0.75);

        } else if (gamepad1.left_trigger != 0 && !gamepad1.right_bumper && !gamepad1.left_bumper) {

            robot.moveBackward(gamepad1.left_trigger);

        } else if (gamepad1.right_trigger != 0 && !gamepad1.right_bumper && !gamepad1.left_bumper) {

            robot.moveForward(gamepad1.right_trigger);

        } else if (gamepad1.left_trigger != 0 && gamepad1.left_bumper) {

            robot.strafeDiagonal("SW", gamepad1.left_trigger);

        } else if (gamepad1.left_trigger != 0 && gamepad1.right_bumper) {

            robot.strafeDiagonal("SE", gamepad1.left_trigger);

        } else if (gamepad1.right_trigger != 0 && gamepad1.left_bumper) {

            robot.strafeDiagonal("NW", gamepad1.right_trigger);

        } else if (gamepad1.right_trigger != 0 && gamepad1.right_bumper) {

            robot.strafeDiagonal("NE", gamepad1.right_trigger);

        } else if (gamepad1.left_stick_x != 0 && gamepad1.left_trigger == 0 && gamepad1.right_trigger == 0) {

            robot.pivot(gamepad1.left_stick_x);

        } else if (gamepad1.left_stick_x != 0 && gamepad1.left_trigger != 0 && gamepad1.right_trigger == 0) {

            robot.turn(-gamepad1.left_trigger, gamepad1.left_stick_x);

        } else if (gamepad1.left_stick_x != 0 && gamepad1.left_trigger == 0 && gamepad1.right_trigger != 0) {

            robot.turn(gamepad1.right_trigger, gamepad1.left_stick_x);

        } else {
            robot.stop();
        }

        telemetry.addData("Top Left Pow", robot.topLeftDrive.getPower());
        telemetry.addData("Bottom Left Pow", robot.bottomLeftDrive.getPower());
        telemetry.addData("Top Right Pow", robot.topRightDrive.getPower());
        telemetry.addData("Bottom Right Pow", robot.bottomRightDrive.getPower());
        telemetry.addData("Heading", robot.angle);
        telemetry.update();

    }
}
