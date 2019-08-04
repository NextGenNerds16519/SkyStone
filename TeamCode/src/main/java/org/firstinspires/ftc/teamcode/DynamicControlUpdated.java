package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "DynamicControlUpdated", group = "")
public class DynamicControlUpdated extends OpMode {

    HardwareDynamicControl robot = new HardwareDynamicControl(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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
        if (gamepad1.right_trigger == 0 || gamepad1.left_trigger == 0) {
            if (gamepad1.left_stick_x > 0) {
                topRight += gamepad1.left_stick_x;
                bottomRight += gamepad1.left_stick_x;
            }
            if (gamepad1.left_stick_x < 0) {
                topLeft += -gamepad1.left_stick_x;
                bottomLeft += -gamepad1.left_stick_x;
            }
        }
        if (gamepad1.right_trigger == 0 || gamepad1.left_trigger != 0) {
            if (gamepad1.left_stick_x > 0) {
                topRight += gamepad1.left_stick_x;
                bottomRight += gamepad1.left_stick_x;
                topLeft += -gamepad1.left_stick_x;
                bottomLeft += -gamepad1.left_stick_x;
            }
            if (gamepad1.left_stick_x < 0) {
                topLeft += -gamepad1.left_stick_x;
                bottomLeft += -gamepad1.left_stick_x;
                topRight += gamepad1.left_stick_x;
                bottomRight += gamepad1.left_stick_x;
            }
        }
        if (gamepad1.right_trigger == 0 && gamepad1.left_trigger == 0 && gamepad1.left_stick_x == 0) {
            if (gamepad1.left_bumper) {
                robot.strafeLeft(1);
            }
            if (gamepad1.right_bumper) {
               robot.strafeRight(1);
            }
        }
        if (gamepad1.left_trigger != 0 && gamepad1.left_bumper) {
            robot.strafeDiagonalLL(1);
        }
        if (gamepad1.right_trigger != 0 && gamepad1.right_bumper) {
            robot.strafeDiagonalRR(1);
        }
        if (gamepad1.left_trigger != 0 && gamepad1.right_bumper) {
            robot.strafeDiagonalLR(1);
        }
        if (gamepad1.right_trigger != 0 && gamepad1.left_bumper) {
            robot.strafeDiagonalRL(1);
        }
        if (!gamepad1.right_bumper&&!gamepad1.left_bumper&&gamepad1.left_trigger==0&&gamepad1.right_trigger==0&&gamepad1.left_stick_x==0&&gamepad1.a){
            //180 code here
        }

        robot.topRightDrive.setPower(topRight);
        robot.bottomRightDrive.setPower(bottomRight);
        robot.topLeftDrive.setPower(topLeft);
        robot.bottomLeftDrive.setPower(bottomLeft);
        telemetry.addData("Top Left Pow", robot.topLeftDrive.getPower());
        telemetry.addData("Bottom Left Pow", robot.bottomLeftDrive.getPower());
        telemetry.addData("Top Right Pow", robot.topRightDrive.getPower());
        telemetry.addData("Bottom Right Pow", robot.bottomRightDrive.getPower());
        telemetry.update();
    }

}
