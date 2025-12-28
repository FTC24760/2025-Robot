package org.firstinspires.ftc.teamcode.teleop;

import static java.lang.Math.abs;
import static java.lang.Math.atan2;
import static java.lang.Math.cos;
import static java.lang.Math.hypot;
import static java.lang.Math.max;
import static java.lang.Math.sin;
import static java.lang.Math.toDegrees;
import static java.lang.Math.toRadians;

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
    private DcMotor leftDrive1, leftDrive2, rightDrive1, rightDrive2;
    private static double MOTOR_REDUCTION = 3;
    private static double SWERVE_REDUCTION = 3;

    Swerve leftSwerve, rightSwerve;

    IMU imu;

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        leftDrive1  = hardwareMap.get(DcMotor.class, "l1Drive");
        leftDrive2  = hardwareMap.get(DcMotor.class, "l2Drive");
        rightDrive1 = hardwareMap.get(DcMotor.class, "r1Drive");
        rightDrive2 = hardwareMap.get(DcMotor.class, "r2Drive");

        leftDrive1.setDirection(DcMotor.Direction.REVERSE);
        leftDrive2.setDirection(DcMotor.Direction.REVERSE);
        rightDrive1.setDirection(DcMotor.Direction.FORWARD);
        rightDrive2.setDirection(DcMotor.Direction.FORWARD);

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);
        leftSwerve = new Swerve(leftDrive1, leftDrive2, toRadians(45));
        rightSwerve = new Swerve(rightDrive1, rightDrive2, toRadians(135));

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

        double driveY    = gamepad1.left_stick_y;
        double driveX    = gamepad1.left_stick_x;
        double driveTurn = gamepad1.right_stick_x;
        double rotatedX = driveX * cos(botHeading) - driveY * sin(botHeading);
        double rotatedY = driveX * sin(botHeading) + driveY * cos(botHeading);

        leftSwerve.update(rotatedX, rotatedY, driveTurn);
        rightSwerve.update(rotatedX, rotatedY, driveTurn);

        telemetry.addData("Status", "Run Time: " + runtime.toString());
    }

    class Swerve {
        DcMotor drive1, drive2;
        double theta;
        double P = 0;
        double I = 0;
        double D = 0;
        double sumError, lastError, lastTime;
        public Swerve(DcMotor drive1, DcMotor drive2, double theta) {//both motors going forwards will rotate the wheel counterclockwise
            this.drive1 = drive1;//going forwards makes the wheel go forwards
            this.drive2 = drive2;//going forwards makes the wheel go backwards
            this.theta = theta;
            sumError = 0;
            lastError = 0;
            lastTime = runtime.milliseconds();
        }
        void reset() {
            drive1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            drive2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            sumError = 0;
            lastError = 0;
            drive1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            drive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        void update(double rotatedX, double rotatedY, double driveTurn) {
            double deltaX = rotatedX + driveTurn * cos(theta + toRadians(90)); // direction of rotation is 90 degrees from direction to center
            double deltaY = rotatedY + driveTurn * sin(theta + toRadians(90));
            double targetAngle = atan2(deltaY, deltaX) % toDegrees(360);
            double targetPower = hypot(deltaY, deltaX);

            double wheelAngle = (drive1.getCurrentPosition() + drive2.getCurrentPosition())
                    / (MOTOR_REDUCTION * SWERVE_REDUCTION);
            if (abs(targetAngle - wheelAngle) > toRadians(90)) {
                targetAngle = (targetAngle + toRadians(180)) % toDegrees(360);
                targetPower *= -1;
            }
            double error = (targetAngle - wheelAngle);
            sumError += error;
            double changeError = (error - lastError)/(runtime.milliseconds() - lastTime);
            lastError = error;
            lastTime = runtime.milliseconds();
            double drive1Power = (P*error) + (I*sumError) + (D*changeError);
            double drive2Power = (P*error) + (I*sumError) + (D*changeError);
            /*if (error < toDegrees(5)) {
                drive1Power += targetPower;
                drive2Power -= targetPower;
            }*/
            if (abs(drive1Power) > 1 || abs(drive2Power) > 1) {
                drive1Power /= max(abs(drive1Power), abs(drive2Power));
                drive2Power /= max(abs(drive1Power), abs(drive2Power));
            }
            drive1.setPower(drive1Power);
            drive2.setPower(drive2Power);
        }
    }
    @Override
    public void stop() {
    }
}
