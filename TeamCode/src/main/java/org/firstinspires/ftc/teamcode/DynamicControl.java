package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "DynamicControl (Blocks to Java)", group = "")
public class DynamicControl extends LinearOpMode {

    private DcMotor topLeftDrive;
    private DcMotor bottomLeftDrive;
    private DcMotor topRightDrive;
    private DcMotor bottomRightDrive;

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        double topLeft;
        double topRight;
        double bottomLeft;
        double bottomRight;

        topLeftDrive = hardwareMap.dcMotor.get("topLeftDrive");
        bottomLeftDrive = hardwareMap.dcMotor.get("bottomLeftDrive");
        topRightDrive = hardwareMap.dcMotor.get("topRightDrive");
        bottomRightDrive = hardwareMap.dcMotor.get("bottomRightDrive");

        // Reverse one of the drive motors.
        // You will have to determine which motor to reverse for your robot.
        // In this example, the right motor was reversed so that positive
        // applied power makes it move the robot in the forward direction.
        topLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        // You will have to determine which motor to reverse for your robot.
        // In this example, the right motor was reversed so that positive
        // applied power makes it move the robot in the forward direction.
        bottomLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();
        bottomLeft = 0;
        topLeft = 0;
        topRight = 0;
        bottomRight = 0;
        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {
                // Put loop blocks here.
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
                if (gamepad1.right_trigger == 0 || gamepad1.left_trigger == 0) {
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
                        bottomLeft = -0.5;
                        bottomRight = 0.5;
                        topLeft = 0.5;
                        topRight = -0.5;
                    }
                    if (gamepad1.right_bumper) {
                        bottomLeft = 0.5;
                        bottomRight = -0.5;
                        topLeft = -0.5;
                        topRight = 0.5;
                    }
                }
                if (gamepad1.left_trigger != 0 && gamepad1.left_bumper) {
                    bottomLeft = 0;
                    bottomRight = 0.5;
                    topLeft = 0.5;
                    topRight = 0;
                }
                if (gamepad1.right_trigger != 0 && gamepad1.right_bumper) {
                    bottomLeft = 0;
                    bottomRight = -0.5;
                    topLeft = -0.5;
                    topRight = 0;
                }
                if (gamepad1.left_trigger != 0 && gamepad1.right_bumper) {
                    bottomLeft = 0.5;
                    bottomRight = 0;
                    topLeft = 0;
                    topRight = 0.5;
                }
                if (gamepad1.right_trigger != 0 && gamepad1.left_bumper) {
                    bottomLeft = -0.5;
                    bottomRight = 0;
                    topLeft = 0;
                    topRight = -0.5;
                }
                topRightDrive.setPower(topRight);
                bottomRightDrive.setPower(bottomRight);
                topLeftDrive.setPower(topLeft);
                bottomLeftDrive.setPower(bottomLeft);
                telemetry.addData("Left Pow", bottomLeftDrive.getPower());
                telemetry.addData("Right Pow", topRightDrive.getPower());
                telemetry.update();
            }
        }
    }
}


