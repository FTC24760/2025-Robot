package org.firstinspires.ftc.teamcode.teleop.oldrobot;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="OLD Flywheel testing")
@Disabled
public class OldFlywheelTesting extends OpMode
{
    DcMotor flywheelLeft;
    DcMotor flywheelRight;

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        flywheelLeft  = hardwareMap.get(DcMotor.class, "leftShooter");
        flywheelRight = hardwareMap.get(DcMotor.class, "rightShooter");

        flywheelRight.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void init_loop() {
    }
    @Override
    public void start() {

    }

    // START-STOP
    @Override
    public void loop() {
        flywheelLeft.setPower(1.0);
        flywheelRight.setPower(1.0);
    }

    @Override
    public void stop() {
    }
}
