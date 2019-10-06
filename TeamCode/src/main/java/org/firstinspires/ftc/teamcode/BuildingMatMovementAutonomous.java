package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Autonomous(name = "BuildingMatMovement", group = "Testing")
public class BuildingMatMovementAutonomous extends LinearOpMode {
    HardwareStrafer robot = new HardwareStrafer(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        waitForStart();

    }
}
