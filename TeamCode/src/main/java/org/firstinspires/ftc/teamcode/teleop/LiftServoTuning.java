package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="LIFT Servo Testing", group="Teleop")
public class LiftServoTuning extends LinearOpMode {

    // Declare OpMode members.


    private ElapsedTime runtime = new ElapsedTime();
    private CRServo liftLeft;
    private CRServo liftRight;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        liftLeft = hardwareMap.get(CRServo.class, "liftLeft");
        liftRight = hardwareMap.get(CRServo.class, "liftRight");

        liftRight.setDirection(DcMotorSimple.Direction.REVERSE);


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            if (gamepad1.a) {
                liftLeft.setPower(1.0);
                liftRight.setPower(1.0);
            } else if (gamepad1.b) {
                liftLeft.setPower(-1.0);
                liftRight.setPower(-1.0);
            } else {
                liftLeft.setPower(0);
                liftRight.setPower(0);
            }

             telemetry.addData("Lift Left Power", liftLeft.getPower());
             telemetry.addData("Lift Right Power", liftRight.getPower());
             telemetry.update();


        }
    }
}
