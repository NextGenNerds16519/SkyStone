package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "NeilsTeleOpAttempt", group = "Neil")
public class NeilsTeleOpAttempt extends OpMode {

    HardwareStraferBasic robot = new HardwareStraferBasic(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    @Override
    public void init() {

        robot.init(hardwareMap);

    }

    @Override
    public void loop() {

        if(gamepad1.right_trigger > 0){
            robot.moveForward(gamepad1.right_trigger);
        } else if(gamepad1.left_trigger > 0){
            robot.moveBackward(gamepad1.left_trigger);
        } else{
            robot.stop();
        }

    }
}
