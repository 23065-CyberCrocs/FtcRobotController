package org.firstinspires.ftc.teamcode.hardware;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

public class Drivetrain extends SubsystemBase {
    private final DcMotorEx[] drivetrainMotors = new DcMotorEx[4];
    // Collection of 4 motors on the drivetrain chassis responsible for powering the base
    public static double Ki = 0.0;
    public static double dKp = 0, dKd = 0, dF = 0;
    private IMU imu;
    static final double COUNTS_PER_REVOLUTION = 28;
    static final double DRIVETRAIN_GEARBOX_RATIO = 12;
    static final double WHEEL_DIAMETER_CM = 7.6;
    static final double COUNTS_PER_CM = (COUNTS_PER_REVOLUTION*DRIVETRAIN_GEARBOX_RATIO)/(WHEEL_DIAMETER_CM*Math.PI);

    public void init(HardwareMap hwMap) {
        for(int i = 0; i < 4; i++) {
            drivetrainMotors[i] = hwMap.get(DcMotorEx.class, "Mec Motor " + i);
            drivetrainMotors[i].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            imu = hwMap.get(IMU.class, "imu");
        }
        // Assigns the 4 motors to the 4 different variables available in the array and runs w/ encoder enabled

        drivetrainMotors[1].setDirection(DcMotorSimple.Direction.REVERSE);
        drivetrainMotors[2].setDirection(DcMotorSimple.Direction.REVERSE);
        drivetrainMotors[3].setDirection(DcMotorSimple.Direction.REVERSE);
        imu.initialize(
                new IMU.Parameters(new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.RIGHT))
        );


    }

    public double getAngle() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    public void drivetrainAlignment() {
        drivetrainMotors[1].setDirection(DcMotorSimple.Direction.REVERSE);
        drivetrainMotors[3].setDirection(DcMotorSimple.Direction.REVERSE);
        refreshEncoders();
    }
    // Sets the top-right wheel and bottom-left wheel in reverse to spin in the same direction as the rest of the drivetrain

    public void frontLeftDrive(double speed) {
        drivetrainMotors[0].setPower(speed);
    }
    public void frontRightDrive(double speed) {
        drivetrainMotors[1].setPower(speed);
    }
    public void backLeftDrive(double speed) {
        drivetrainMotors[2].setPower(speed);
    }
    public void backRightDrive(double speed) {
        drivetrainMotors[3].setPower(speed);
    }

    // Allows individual motors to be controlled separately


    public void rightStrafe(double speed) {
        drivetrainMotors[0].setPower(speed);
        drivetrainMotors[1].setPower(-speed);
        drivetrainMotors[2].setPower(-speed);
        drivetrainMotors[3].setPower(speed);
    }

    public void leftStrafe(double speed) {
        drivetrainMotors[0].setPower(-speed);
        drivetrainMotors[1].setPower(speed);
        drivetrainMotors[2].setPower(speed);
        drivetrainMotors[3].setPower(-speed);
    }

    public void linearDrive(double speed) {
        drivetrainMotors[0].setPower(speed);
        drivetrainMotors[1].setPower(speed);
        drivetrainMotors[2].setPower(speed);
        drivetrainMotors[3].setPower(speed);
    }


    public void rotate(double power) {
        drivetrainMotors[0].setPower(power);
        drivetrainMotors[1].setPower(-power);
        drivetrainMotors[2].setPower(power);
        drivetrainMotors[3].setPower(-power);

    }

    public void linearDriveByCM(double centimetres, double power) {
        int ticksTravelled = (int) Math.round(centimetres*COUNTS_PER_CM);
        for (DcMotor motor : drivetrainMotors) {
            motor.setTargetPosition(motor.getCurrentPosition() + ticksTravelled);
            motor.setPower(power);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }

    public void leftStrafeByCM(double CM, double power) {
        int ticksTravelled = Math.round(Math.round(CM*COUNTS_PER_CM));
        drivetrainMotors[0].setTargetPosition(-ticksTravelled);
        drivetrainMotors[1].setTargetPosition(ticksTravelled);
        drivetrainMotors[2].setTargetPosition(ticksTravelled);
        drivetrainMotors[3].setTargetPosition(-ticksTravelled);
        for (DcMotor motor : drivetrainMotors) {
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }
    public void rightStrafeByCM(double CM, double power) {
        int ticksTravelled = Math.round(Math.round(CM*COUNTS_PER_CM));
        drivetrainMotors[0].setTargetPosition(ticksTravelled);
        drivetrainMotors[0].setPower(power);
        drivetrainMotors[1].setTargetPosition(-ticksTravelled);
        drivetrainMotors[1].setPower(power);
        drivetrainMotors[2].setTargetPosition(-ticksTravelled);
        drivetrainMotors[2].setPower(power);
        drivetrainMotors[3].setTargetPosition(ticksTravelled);
        drivetrainMotors[3].setPower(power);
        for (DcMotor motor : drivetrainMotors) {
            motor.setPower(power);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }

    public void disableEncoders() {
        for (DcMotor motor : drivetrainMotors) {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    public void refreshEncoders() {
        for (DcMotor motor : drivetrainMotors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }

    public boolean isBusy() {
        return drivetrainMotors[0].isBusy() || drivetrainMotors[1].isBusy() || drivetrainMotors[2].isBusy() || drivetrainMotors[3].isBusy();
    }

    public void enableRTP() {
        for (DcMotor motor : drivetrainMotors) {
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }


    public double getRobotOrientation() {
        return imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS).firstAngle;
    }

    public void resetYaw() {
        imu.resetYaw();
    }
}