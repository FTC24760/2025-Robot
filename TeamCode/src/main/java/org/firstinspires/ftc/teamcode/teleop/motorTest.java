package org.firstinspires.ftc.teamcode.teleop;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="motor test")
public class motorTest extends OpMode {
    DcMotor r1, r2;
    @Override
    public void init() {
        r1 = hardwareMap.get(DcMotor.class, "r1");
        r2 = hardwareMap.get(DcMotor.class, "r2");

    }

    @Override
    public void loop() {
        r1.setPower(gamepad1.left_stick_y);
        r2.setPower(gamepad1.right_stick_y);
    }

}
