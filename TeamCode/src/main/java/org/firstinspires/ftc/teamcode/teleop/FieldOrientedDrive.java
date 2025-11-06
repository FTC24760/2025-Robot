package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name="Field Oriented Drive")
public class FieldOrientedDrive extends OpMode
{
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    IMU imu;

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        leftFrontDrive  = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive  = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive  = hardwareMap.get(DcMotor.class, "right_back_drive");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void init_loop() {
    }
    @Override
    public void start() {
        runtime.reset();
    }

    // START-STOP
    @Override
    public void loop() {
        double leftFrontPower;
        double leftBackPower;
        double rightFrontPower;
        double rightBackPower;

        if (gamepad1.start) { // change this to a button
            imu.resetYaw();
        }
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        double driveY    = gamepad1.left_stick_y;
        double driveX    = gamepad1.left_stick_x;
        double driveTurn = gamepad1.right_stick_x;

        double rotatedX = driveX * Math.cos(botHeading) - driveY * Math.sin(botHeading);
        double rotatedY = driveX * Math.sin(botHeading) + driveY * Math.cos(botHeading);

        double denominator = Math.max(Math.abs(rotatedY) + Math.abs(rotatedX) + Math.abs(driveTurn), 1);
        leftFrontPower  = (rotatedY  + rotatedX + driveTurn) / denominator;
        leftBackPower   = (rotatedY  - rotatedX + driveTurn) / denominator;
        rightFrontPower = (-rotatedY - rotatedX - driveTurn) / denominator;
        rightBackPower  = (-rotatedY + rotatedX - driveTurn) / denominator;

        leftFrontDrive.setPower(leftFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightFrontDrive.setPower(rightFrontPower);
        rightBackDrive.setPower(rightBackPower);

        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "left front (%.2f) back (%.2f), right front (%.2f) back (%.2f)",
                leftFrontPower, leftBackPower, rightFrontPower, rightBackPower);
    }

    @Override
    public void stop() {
    }
}
