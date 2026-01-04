package org.firstinspires.ftc.teamcode.teleop;

import static org.firstinspires.ftc.teamcode.SwerveRotationalPID.R_D;
import static org.firstinspires.ftc.teamcode.SwerveRotationalPID.R_I;
import static org.firstinspires.ftc.teamcode.SwerveRotationalPID.R_P;
import static java.lang.Math.abs;
import static java.lang.Math.atan2;
import static java.lang.Math.cos;
import static java.lang.Math.hypot;
import static java.lang.Math.max;
import static java.lang.Math.sin;
import static java.lang.Math.toDegrees;
import static java.lang.Math.toRadians;


import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name="Non Field Oriented Drive- Swerve")
public class NonFieldOrientedDrive extends OpMode
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

        //leftSwerve.update(rotatedX, rotatedY, driveTurn);
        rightSwerve.update(rotatedX, rotatedY, driveTurn);
        telemetry.addData("Drive", "x: %.2f, y: %.2f", driveX, driveY);
        telemetry.addData("Left Swerve", "Target: %.2f, Angle: %.2f", toDegrees(leftSwerve.targetAngle), toDegrees(leftSwerve.wheelAngle));
        telemetry.addData("Right Swerve", "Target: %.2f, Angle: %.2f", toDegrees(rightSwerve.targetAngle), toDegrees(rightSwerve.wheelAngle));
        telemetry.addData("Motor Encoders", "%d %d", rightDrive1.getCurrentPosition(), rightDrive2.getCurrentPosition());
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.update();
    }

    class Swerve {
        private double MOTOR_REDUCTION = 28 * 2.89  * 3.61;
        private double SWERVE_REDUCTION = 6.25;
        private double SPEED_COEFFICIENT = 1;
        DcMotor drive1, drive2;
        double theta;
        double targetAngle, wheelAngle, wheelSpeed, integralRotE, derivativeRotE, lastE = 0;
        double lastWheelRotation, lastWheelSpeed, speedE, integralSpeedE, derivativeSpeedE = 0;
        double lastTime;
        public Swerve(DcMotor drive1, DcMotor drive2, double theta) {//both motors going forwards will rotate the wheel counterclockwise
            this.drive1 = drive1;//going forwards makes the wheel go forwards
            this.drive2 = drive2;//going forwards makes the wheel go backwards
            this.theta = theta;
            lastTime = runtime.milliseconds();
            drive1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            drive2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            drive1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            drive2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        }
        void reset() {
            drive1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            drive2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            integralRotE = 0;
            lastE = 0;
            drive1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            drive2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        void update(double rotatedX, double rotatedY, double driveTurn) {
            // -------------------------------- KINEMATICS --------------------------------
            double deltaX = rotatedX + driveTurn * cos(theta + toRadians(90)); // direction of rotation is 90 degrees from direction to center
            double deltaY = rotatedY + driveTurn * sin(theta + toRadians(90));
            double targetSpeed = hypot(deltaY, deltaX) * SPEED_COEFFICIENT;
            //targetSpeed = targetSpeed + (1-abs(targetSpeed))
            wheelAngle = a360((drive1.getCurrentPosition() + drive2.getCurrentPosition()) * 0.5
                              / MOTOR_REDUCTION / SWERVE_REDUCTION * toRadians(360));

            if (targetSpeed > 0.01)
                targetAngle = a360(atan2(deltaY, deltaX) - toRadians(90));
            else
                targetAngle = theta;
            
            double rotE = a180(targetAngle - wheelAngle);
            targetSpeed *= cos(rotE); // cosine correction

            if (abs(rotE) > toRadians(90))
                rotE = a180(rotE + toRadians(180));

            // -------------------------------- PID --------------------------------
            integralRotE += rotE;
            derivativeRotE = (rotE - lastE)/(runtime.milliseconds() - lastTime);

            double rotationPower = (R_P * rotE) + (R_I * integralRotE) + (R_D *derivativeRotE);

            /*double wheelRotation = (drive1.getCurrentPosition() - drive2.getCurrentPosition())
                    / MOTOR_REDUCTION * toRadians(360);
            wheelSpeed = (wheelRotation - lastWheelRotation) / (runtime.milliseconds() - lastTime);
            speedE = targetSpeed - wheelSpeed;

            integralSpeedE += speedE;
            derivativeSpeedE = (wheelSpeed - lastWheelSpeed) / (runtime.milliseconds() - lastTime);
            double translationPower = (T_P * speedE) + (T_I * integralSpeedE) + (T_D * derivativeSpeedE);

            if (abs(rotationPower) + abs(translationPower) > 1)
                translationPower *= (1 - abs(rotationPower)) / abs(translationPower);*/
            double translationPower = targetSpeed;
            drive1.setPower(rotationPower + translationPower);
            drive2.setPower(rotationPower - translationPower);

            lastE = rotE;
            lastTime = runtime.milliseconds();
            //lastWheelRotation = wheelRotation;
            //lastWheelSpeed = wheelSpeed;

        }
    }
    @Override
    public void stop() {
    }
    double a360(double angle) {
        while (angle > toRadians(360)) angle -= toRadians(360);
        while (angle < 0) angle += toRadians(360);
        return angle;
    }
    double a180(double angle) {
        while (angle > toRadians(180)) angle -= toRadians(360);
        while (angle < -180) angle += toRadians(360);
        return angle;
    }
}
