package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.concurrent.TimeUnit;

@TeleOp(name = "StraferTeleOp", group = "Final")
public class StraferTeleOp extends OpMode {

    HardwareStrafer robot = new HardwareStrafer(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    public double topLeft;
    public double topRight;
    public double bottomLeft;
    public double bottomRight;

    @Override
    public void init(){
        robot.init(hardwareMap);

    }

    @Override
    public void loop(){

        float[] leftStick = {gamepad1.left_stick_x, gamepad1.left_stick_y};

        if (gamepad1.right_trigger == 0 && gamepad1.left_trigger == 0 && !gamepad1.left_bumper && !gamepad1.right_bumper && leftStick[0] == 0 && leftStick[1] == 0 && topRight!=0 && topLeft!=0 && bottomRight !=0 && bottomLeft != 0){
            robot.bottomLeftDrive.setPower(-bottomLeft);
            robot.bottomRightDrive.setPower(-bottomRight);
            robot.topLeftDrive.setPower(-topLeft);
            robot.topRightDrive.setPower(-topRight);
            try {
                TimeUnit.MILLISECONDS.sleep(500);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            robot.stop();

        } else{
            topLeft = robot.topLeftDrive.getPower();
            topRight = robot.topRightDrive.getPower();
            bottomLeft = robot.bottomLeftDrive.getPower();
            bottomRight = robot.bottomRightDrive.getPower();
        }

        robot.betterControls(gamepad1.right_trigger, gamepad1.left_trigger, leftStick, gamepad1.right_bumper, gamepad1.left_bumper);

        telemetry.addData("Top Left Pow", robot.topLeftDrive.getPower());
        telemetry.addData("Bottom Left Pow", robot.bottomLeftDrive.getPower());
        telemetry.addData("Top Right Pow", robot.topRightDrive.getPower());
        telemetry.addData("Bottom Right Pow", robot.bottomRightDrive.getPower());
        telemetry.addData("Heading", robot.angle);
        telemetry.update();




    }
}
