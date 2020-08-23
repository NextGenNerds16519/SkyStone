package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "BuildingMatMovement", group = "Testing")

public class AutoPeriod extends LinearOpMode {

    HardwareStrafer robot = new HardwareStrafer(DcMotor.RunMode.RUN_USING_ENCODER);
    int color = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);
        waitForStart();
        robot.moveDistance(1, 48);
        robot.grabMat();
        robot.moveDistance(-1, -48);
        robot.releaseMat();
        robot.strafeLeftAuto(1, 1000);
        robot.moveDistance(1, 12);
        int[] lineColor = robot.getLineColor();
        telemetry.addData("lineColor", lineColor);

        while(!(lineColor[color] > 200)){
            robot.strafeLeftAuto(1, 10);
            lineColor = robot.getLineColor();
        }
        robot.stop();

    }

}