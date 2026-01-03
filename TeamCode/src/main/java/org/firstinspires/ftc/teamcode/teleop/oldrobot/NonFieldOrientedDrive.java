package org.firstinspires.ftc.teamcode.teleop.oldrobot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Regular Drive")
public class NonFieldOrientedDrive extends OpMode
{
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        leftFrontDrive  = hardwareMap.get(DcMotor.class, "flDrive");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "rlDrive");
        rightFrontDrive  = hardwareMap.get(DcMotor.class, "frDrive");
        rightBackDrive  = hardwareMap.get(DcMotor.class, "rrDrive");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);


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


        double y    = -gamepad1.left_stick_y;
        double x    = gamepad1.left_stick_x * 1.1;
        double rx = gamepad1.right_stick_x;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        leftFrontPower = (y + x + rx) / denominator;
        leftBackPower = (y - x + rx) / denominator;
        rightFrontPower = (y - x - rx) / denominator;
        rightBackPower = (y + x - rx) / denominator;

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
