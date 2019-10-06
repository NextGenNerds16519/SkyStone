package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous (name = "NeilsAutoOp", group = "Neil")
public class NeilsAutoOp extends OpMode {

    HardwareStraferBasic robot = new HardwareStraferBasic(DcMotor.RunMode.RUN_USING_ENCODER);

    @Override
    public void init() {

        robot.init(hardwareMap);

    }

    @Override
    public void loop() {

        moveDistance(1, 10);''

    }

    public void moveDistance(float power, int distance){
        //Reset Encoders
        robot.bottomLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.bottomRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.topLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.topRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Set target position
        robot.bottomLeftDrive.setTargetPosition(distance);
        robot.bottomRightDrive.setTargetPosition(distance);
        robot.topLeftDrive.setTargetPosition(distance);
        robot.topRightDrive.setTargetPosition(distance);

        //Set mode run to position
        robot.bottomLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.bottomRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.topLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.topRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while(robot.bottomLeftDrive.isBusy() && robot.bottomRightDrive.isBusy() && robot.topLeftDrive.isBusy() && robot.topRightDrive.isBusy())
    }
    public void move(float power) {

        robot.bottomLeftDrive.setPower(power);
        robot.bottomRightDrive.setPower(power);
        robot.topLeftDrive.setPower(power);
        robot.topRightDrive.setPower(power);

    }

}
