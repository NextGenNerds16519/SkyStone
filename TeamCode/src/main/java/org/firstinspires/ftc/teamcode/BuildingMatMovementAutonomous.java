package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "BuildingMatMovement", group = "Testing")
public class BuildingMatMovementAutonomous extends LinearOpMode {
    HardwareStrafer robot = new HardwareStrafer(DcMotor.RunMode.RUN_USING_ENCODER);

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        waitForStart();
        robot.moveDistance(1,6.5);
        robot.strafeLeftAuto(1,750);
        robot.moveDistance(-1,4);

    }
}