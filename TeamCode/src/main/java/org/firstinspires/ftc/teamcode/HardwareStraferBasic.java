package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class HardwareStraferBasic {

    public DcMotor bottomLeftDrive;
    public DcMotor bottomRightDrive;
    public BNO055IMU imu;
    public DcMotor topLeftDrive;
    public DcMotor topRightDrive;

    private DcMotor.RunMode initialMode;

    HardwareMap map;

    public HardwareStraferBasic(DcMotor.RunMode enteredMode) {

        initialMode = enteredMode;

    }

    public void init(HardwareMap aMap) {

        map = aMap;

        bottomLeftDrive = map.dcMotor.get("bottomLeftDrive");
        bottomRightDrive = map.dcMotor.get("bottomRightDrive");
        imu = map.get(BNO055IMU.class, "imu");
        topLeftDrive = map.dcMotor.get("topLeftDrive");
        topRightDrive = map.dcMotor.get("topRightDrive");

        //Encoders
        bottomLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottomRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        topLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        topRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //set mode
        bottomLeftDrive.setMode(initialMode);
        bottomRightDrive.setMode(initialMode);
        topLeftDrive.setMode(initialMode);
        topRightDrive.setMode(initialMode);

        //set zero power mode
        bottomLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bottomRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        topLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        topRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //set direction
        bottomLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        bottomRightDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        topLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        topRightDrive.setDirection(DcMotorSimple.Direction.FORWARD);

        //

        stop();

    }

    public void stop() {

        bottomLeftDrive.setPower(0);
        bottomRightDrive.setPower(0);
        topLeftDrive.setPower(0);
        topRightDrive.setPower(0);

    }

    public void strafeLeft(float power) {

        bottomLeftDrive.setPower(-power);
        bottomRightDrive.setPower(power);
        topLeftDrive.setPower(power);
        topRightDrive.setPower(-power);

    }

    public void strafeRight(float power) {

        bottomLeftDrive.setPower(power);
        bottomRightDrive.setPower(-power);
        topLeftDrive.setPower(-power);
        topRightDrive.setPower(power);

    }

    public void moveForward(float power) {

        bottomLeftDrive.setPower(power);
        bottomRightDrive.setPower(power);
        topLeftDrive.setPower(power);
        topRightDrive.setPower(power);

    }

    public void moveBackward(float power) {

        bottomLeftDrive.setPower(-power);
        bottomRightDrive.setPower(-power);
        topLeftDrive.setPower(-power);
        topRightDrive.setPower(-power);

    }

    public void pivot(float power) {

        bottomLeftDrive.setPower(0);
        bottomRightDrive.setPower(0);
        topLeftDrive.setPower(0);
        topRightDrive.setPower(0);

    }

    public void turnLeft(float power) {

        bottomLeftDrive.setPower(0);
        bottomRightDrive.setPower(0);
        topLeftDrive.setPower(0);
        topRightDrive.setPower(0);

    }

    public void turnRight(float power) {

        bottomLeftDrive.setPower(0);
        bottomRightDrive.setPower(0);
        topLeftDrive.setPower(0);
        topRightDrive.setPower(0);

    }

    public void turnLeft(int degrees) {

        bottomLeftDrive.setPower(0);
        bottomRightDrive.setPower(0);
        topLeftDrive.setPower(0);
        topRightDrive.setPower(0);

    }

    public void turnRight(int degrees) {

        bottomLeftDrive.setPower(0);
        bottomRightDrive.setPower(0);
        topLeftDrive.setPower(0);
        topRightDrive.setPower(0);

    }

}

