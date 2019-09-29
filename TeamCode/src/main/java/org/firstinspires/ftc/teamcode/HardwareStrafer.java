package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.bosch.BNO055IMU;
//import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.GyroSensor;
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
//    public int zAccumulated;
//    public int heading;
//    public int xVal, yVal, zVal;
//    GyroSensor sensorGyro;
//    ModernRoboticsI2cGyro mrGyro;


    private DcMotor.RunMode initialMode;

    HardwareMap map;

    public HardwareStrafer(DcMotor.RunMode enteredMode) {

        initialMode = enteredMode;

    }

    public void init(HardwareMap aMap) /*throws InterruptedException*/{

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

//        //gyro
//        sensorGyro = map.gyroSensor.get("gyro");
//        mrGyro = (ModernRoboticsI2cGyro) sensorGyro;
//        mrGyro.calibrate();
//        wait(4000)a;
//
//        stop();

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

    public void turnAngle(float power, int degrees){
        float heading = getAngle();
        float target = heading + degrees;
        while (Math.abs(getAngle()-target)>3) {

            if (getAngle() > target) {
                pivot(power);
            } else if (getAngle() < target) {
                pivot(-power);
            }
        }
        stop();
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

    public void pivot(float power) {

        bottomLeftDrive.setPower(power);
        bottomRightDrive.setPower(-power);
        topLeftDrive.setPower(power);
        topRightDrive.setPower(-power);

    }



    public void totalControl(float rightTrigger, float leftTrigger, float[] leftStick, boolean rightBumper, boolean leftBumper){
        float frontLeft = rightTrigger - leftTrigger;
        float backLeft = rightTrigger - leftTrigger;
        float frontRight = rightTrigger - leftTrigger;
        float backRight = rightTrigger - leftTrigger;

        if(rightTrigger == 0 && leftTrigger == 0 && !rightBumper && !leftBumper && leftStick[0] == 0 && leftStick[1] == 0){
            frontLeft = 0;
            backLeft = 0;
            frontRight = 0;
            backRight = 0;
        }
        if(rightBumper){
            strafeRight(1);
            return;
        } else if(leftBumper){
            strafeLeft(1);
            return;
        }

        if(leftStick[0] > 0){
            if(frontLeft != 0 && backLeft != 0) {
                frontRight *= (1 - leftStick[0]);
                backRight *= (1 - leftStick[0]);
            } else {
                // Pivot
                frontLeft = leftStick[0];
                backLeft = leftStick[0];
                frontRight = -leftStick[0];
                backRight = -leftStick[0];
            }
        } else if(leftStick[0] < 0){
            if(frontRight != 0 && backRight != 0){
                frontLeft *= (1 + leftStick[0]);
                backLeft *= (1 + leftStick[0]);
            } else {
                // Pivot
                // Note that leftStick[0] is currently positive
                frontLeft = leftStick[0];
                backLeft = leftStick[0];
                frontRight = -leftStick[0];
                backRight = -leftStick[0];
            }
        }
        // Trigger controls y power
        // Left stick degree controls x movement power
        bottomLeftDrive.setPower(backLeft);
        bottomRightDrive.setPower(backRight);
        topLeftDrive.setPower(frontLeft);
        topRightDrive.setPower(frontRight);
    }




    public void betterControls(float rightTrigger, float leftTrigger, float[] leftStick, boolean rightBumper, boolean leftBumper){
        float frontLeft = rightTrigger - leftTrigger;
        float backLeft = rightTrigger - leftTrigger;
        float frontRight = rightTrigger - leftTrigger;
        float backRight = rightTrigger - leftTrigger;

        if(rightTrigger == 0 && leftTrigger == 0 && !rightBumper && !leftBumper && leftStick[0] == 0 && leftStick[1] == 0){
            stop();
        }

        if (rightTrigger !=0 && rightBumper){
            strafeDiagonal("NE",1);
        } else if (rightTrigger !=0 && leftBumper){
            strafeDiagonal("NW",1);
        } else if (leftTrigger !=0 && rightBumper){
            strafeDiagonal("SE",1);
        } else if (leftTrigger !=0 && leftBumper){
            strafeDiagonal("SW",1);
        } else if(rightBumper){
            strafeRight(1);
            return;
        } else if(leftBumper){
            strafeLeft(1);
            return;
        }

        if(leftStick[0] > 0){
            if(frontLeft != 0 && backLeft != 0) {
                frontRight *= (1 - leftStick[0]);
                backRight *= (1 - leftStick[0]);
            } else {
                // Pivot
                frontLeft = leftStick[0];
                backLeft = leftStick[0];
                frontRight = -leftStick[0];
                backRight = -leftStick[0];
            }
        } else if(leftStick[0] < 0){
            if(frontRight != 0 && backRight != 0){
                frontLeft *= (1 + leftStick[0]);
                backLeft *= (1 + leftStick[0]);
            } else {
                // Pivot
                // Note that leftStick[0] is currently positive
                frontLeft = leftStick[0];
                backLeft = leftStick[0];
                frontRight = -leftStick[0];
                backRight = -leftStick[0];
            }
        }
        // Trigger controls y power
        // Left stick degree controls x movement power
        bottomLeftDrive.setPower(backLeft);
        bottomRightDrive.setPower(backRight);
        topLeftDrive.setPower(frontLeft);
        topRightDrive.setPower(frontRight);
    }


//    public void turnGyro(float power,int angle) throws InterruptedException{
//        //setup
//        angle = angle + mrGyro.getIntegratedZValue();
//        zAccumulated = mrGyro.getIntegratedZValue();
//        heading = 360 - mrGyro.getHeading();
//
//        if (heading==360){
//            heading =0;
//        }
//
//        xVal = mrGyro.rawX();
//        yVal = mrGyro.rawY();
//        zVal = mrGyro.rawZ();
//
//        //turn
//        while (Math.abs(zAccumulated-target)>3) {
//
//            if (zAccumulated > angle) {
//                pivot(1);
//            } else if (zAccumulated < angle) {
//                pivot(-1);
//            }
//            wait(3000);
//        }
//        stop();
//        wait(1000);
//
//
//    }





}

