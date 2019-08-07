package org.firstinspires.ftc.teamcode;

import android.graphics.drawable.GradientDrawable;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class HardwareStrafer {

    public DcMotor bottomLeftDrive;
    public DcMotor bottomRightDrive;
    public BNO055IMU imu;
    public DcMotor topLeftDrive;
    public DcMotor topRightDrive;
    public Orientation angle;

    private DcMotor.RunMode initialMode;

    HardwareMap map;

    public HardwareStrafer(DcMotor.RunMode enteredMode) {

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
        bottomLeftDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        bottomRightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        topLeftDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        topRightDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        stop();

        //imu set up
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        imu.initialize(parameters);
        angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

    }

    public void stop() {

        bottomLeftDrive.setPower(0);
        bottomRightDrive.setPower(0);
        topLeftDrive.setPower(0);
        topRightDrive.setPower(0);

    }

    public float getAngle() {
        angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angle.firstAngle;
    }

    public void strafeLeft(double power) {

        bottomLeftDrive.setPower(power);
        bottomRightDrive.setPower(-power);
        topLeftDrive.setPower(-power);
        topRightDrive.setPower(power);

    }

    public void strafeRight(double power) {

        bottomLeftDrive.setPower(-power);
        bottomRightDrive.setPower(power);
        topLeftDrive.setPower(power);
        topRightDrive.setPower(-power);

    }

    public void strafeDiagonal(String direction, float power) {
        switch (direction) {
            case "NE":

                bottomLeftDrive.setPower(0);
                bottomRightDrive.setPower(power);
                topLeftDrive.setPower(power);
                topRightDrive.setPower(0);
                break;

            case "SE":

                bottomLeftDrive.setPower(-power);
                bottomRightDrive.setPower(0);
                topLeftDrive.setPower(0);
                topRightDrive.setPower(-power);
                break;

            case "SW":

                bottomLeftDrive.setPower(0);
                bottomRightDrive.setPower(-power);
                topLeftDrive.setPower(-power);
                topRightDrive.setPower(0);
                break;

            case "NW":

                bottomLeftDrive.setPower(power);
                bottomRightDrive.setPower(0);
                topLeftDrive.setPower(0);
                topRightDrive.setPower(power);
                break;
        }
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

        bottomLeftDrive.setPower(power);
        bottomRightDrive.setPower(-power);
        topLeftDrive.setPower(power);
        topRightDrive.setPower(-power);

    }

    public void turn(float power, float turn) {

        turn = turn + 1;
        turn = turn / 2;

        bottomLeftDrive.setPower(turn * power);
        bottomRightDrive.setPower((1 - turn) * power);
        topLeftDrive.setPower(turn * power);
        topRightDrive.setPower((1 - turn) * power);

    }

    public void betterTurn(float power, float turn){
        float leftpower= power;
        float rightpower = power;
        if (turn<0){
            leftpower = leftpower+turn;
            topLeftDrive.setPower(leftpower);
            bottomLeftDrive.setPower(leftpower);
        }

        if (turn>0){
            rightpower = rightpower-turn;
            topRightDrive.setPower(rightpower);
            bottomRightDrive.setPower(rightpower);
        }
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

