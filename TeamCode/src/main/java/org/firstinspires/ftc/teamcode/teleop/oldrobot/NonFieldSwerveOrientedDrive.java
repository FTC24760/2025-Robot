package org.firstinspires.ftc.teamcode.teleop.oldrobot;

import static java.lang.Math.toDegrees;
import static java.lang.Math.toRadians;


import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Swerve;

@TeleOp(name="NonField Swerve Oriented Drive")
@Disabled
public class NonFieldSwerveOrientedDrive extends OpMode
{
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive1, leftDrive2, rightDrive1, rightDrive2;

    Swerve leftSwerve, rightSwerve;

    IMU imu;

    @Override
    public void init() {
        leftDrive1  = hardwareMap.get(DcMotor.class, "l1");
        leftDrive2  = hardwareMap.get(DcMotor.class, "l2");
        rightDrive1 = hardwareMap.get(DcMotor.class, "r1");
        rightDrive2 = hardwareMap.get(DcMotor.class, "r2");
        leftDrive1.setDirection(DcMotorSimple.Direction.REVERSE);
        leftDrive2.setDirection(DcMotorSimple.Direction.REVERSE);
        rightDrive1.setDirection(DcMotorSimple.Direction.REVERSE);
        rightDrive2.setDirection(DcMotorSimple.Direction.REVERSE);
        leftDrive1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDrive2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);
        leftSwerve = new Swerve(leftDrive1, leftDrive2, toRadians(45));
        rightSwerve = new Swerve(rightDrive2, rightDrive1, toRadians(135));

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
        if (gamepad1.start) { // change this to a button
            imu.resetYaw();
        }
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        double driveY    = -gamepad1.left_stick_y;
        double driveX    = gamepad1.left_stick_x;
        double driveTurn = gamepad1.right_stick_x;
        double rotatedX = driveX;//driveX * cos(botHeading) - driveY * sin(botHeading);
        double rotatedY = driveY;//driveX * sin(botHeading) + driveY * cos(botHeading);

        leftSwerve.update(rotatedX, rotatedY, driveTurn);
        rightSwerve.update(rotatedX, rotatedY, driveTurn);
        telemetry.addData("Drive", "x: %.2f, y: %.2f", driveX, driveY);
        telemetry.addData("Left Swerve", "Target: %.2f, Angle: %.2f", toDegrees(leftSwerve.targetAngle), toDegrees(leftSwerve.wheelAngle));
        telemetry.addData("Right Swerve", "Target: %.2f, Angle: %.2f", toDegrees(rightSwerve.targetAngle), toDegrees(rightSwerve.wheelAngle));
        telemetry.addData("Motor Encoders", "%d %d", rightDrive1.getCurrentPosition(), rightDrive2.getCurrentPosition());
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.update();
    }


    @Override
    public void stop() {
    }

}
