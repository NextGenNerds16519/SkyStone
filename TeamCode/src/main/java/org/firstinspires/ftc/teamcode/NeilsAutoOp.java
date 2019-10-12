package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name = "NeilsAutoOp", group = "Neil")
public class NeilsAutoOp extends LinearOpMode {

    NeilsTestingHardwareStrafer robot = new NeilsTestingHardwareStrafer(DcMotor.RunMode.RUN_USING_ENCODER);

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        float power = 0.25f;
        robot.moveDistance(power, 12);
        robot.moveDistance(-power, -12);
    }
}
