package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name = "NeilsTeleOpAttempt", group = "Neil")
public class NeilsTeleOpAttempt extends OpMode {

    HardwareStraferBasic robot = new HardwareStraferBasic(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    @Override
    public void init() {

        robot.init(hardwareMap);

    }

    @Override
    public void loop() {

        if (gamepad1.right_trigger > 0) {
            robot.moveForward(gamepad1.right_trigger);
        } else if (gamepad1.left_trigger > 0) {
            robot.moveBackward(gamepad1.left_trigger);
        } else if (gamepad1.right_bumper) {
            robot.strafeRight((float) 0.75);
        } else if (gamepad1.left_bumper) {
            robot.strafeLeft((float) 0.75);
        } else if (gamepad1.left_stick_x != 0) {
            robot.pivot(gamepad1.left_stick_x);
        } else if (gamepad1.a) {

            float targetAngle = robot.angle.firstAngle + 90;
            while (robot.angle.firstAngle != targetAngle) {
                robot.pivot((float) 0.5);
            }
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
