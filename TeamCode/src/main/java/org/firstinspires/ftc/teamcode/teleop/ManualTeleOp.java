package org.firstinspires.ftc.teamcode.teleop;

import com.pedropathing.util.Timer;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo; // <--- IMPORT ADDED
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@TeleOp(name="Manual TeleOp", group="Competition")
public class ManualTeleOp extends DualCameraTeleOp {
    private int scoringState;
    Timer actionTimer;

    @Override
    public void runOpMode() {
        initHardware();
        initLogic();

        telemetry.addData("Status", "Initialized. Preloaded: P, P, G");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // 1. INPUTS
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.1;
            double rx = gamepad1.right_stick_x;

            driveFieldCentric(y, x, rx);
            if (gamepad1.dpad_left || gamepad2.dpad_left) {
                int s = getSlotWithColor("Purple");
                if (s != -1) targetSlotIndex = s;
            }
            if (gamepad1.dpad_right || gamepad2.dpad_right) {
                int s = getSlotWithColor("Green");
                if (s != -1) targetSlotIndex = s;
            }
            // 2. STATE HANDLING
            switch (currentState) {
                case DRIVER_CONTROL:
                    // Ensure intake is off during normal driving
                    intakeSpinner.setPower(0.0);

                    //driveFieldCentric(y, x, rx);

                    if (gamepad1.right_bumper || gamepad2.right_bumper) {
                        targetSlotIndex = getNextEmptySlot();
                        if (targetSlotIndex != -1) {
                            driveDirection = 1.0;
                            currentState = RobotState.AUTO_ALIGN_INTAKE;
                        }
                    }

                    if (gamepad1.left_bumper || gamepad2.left_bumper) {
                        String requiredColor = motif.get(motifIndex);
                        /*targetSlotIndex = getSlotWithColor(requiredColor);

                        if (targetSlotIndex != -1) {
                            driveDirection = -1.0;
                            currentState = RobotState.AUTO_ALIGN_SCORE;
                        } else {
                            telemetry.addData("Alert", "No " + requiredColor + " found!");
                        }*/
                        scoringState = -1;
                        currentState = RobotState.AUTO_ALIGN_SCORE;
                    }

                    if (gamepad1.right_trigger > 0.3) {
                        imu.resetYaw();
                    }
                    break;

                case AUTO_ALIGN_INTAKE:
                    runIntakeLogic();
                    break;

                case AUTO_ALIGN_SCORE:
                    runScoreLogic();
                    break;
            }

            // 3. HARDWARE UPDATES
            updateRevolverServos();

            // 4. TELEMETRY
            telemetry.addData("Slot 1", slots.get(0).color);
            telemetry.addData("Slot 2", slots.get(1).color);
            telemetry.addData("Slot 3", slots.get(2).color);
            telemetry.addData("Mode", currentState);
            telemetry.addData("Motif Need", motif.get(motifIndex));
            telemetry.addData("Drive Dir", driveDirection > 0 ? "FWD (Intake)" : "REV (Score)");
            telemetry.update();
        }
    }

    // ==========================================================================
    //                         INTAKE LOGIC (Limelight - Front)
    // ==========================================================================
    private void runIntakeLogic() {
        // 1. Spin Intake Active
        intakeSpinner.setPower(0.5); // <--- NEW: Spins 'in'
        // 2. Servo Setup
        if (targetSlotIndex != -1)
            revolverServo.setPosition(INTAKE_POSITIONS[targetSlotIndex]);
        slots.get(targetSlotIndex).isClawOpen = true;

        /*LLResult result = limelight.getLatestResult();
        double turnPower = 0;
        double drivePower = 0; // <--- NEW: Forward/Back Power
        String detectedLabel = "Unknown";

        if (result != null && result.isValid()) {
            List<LLResultTypes.DetectorResult> detections = result.getDetectorResults();
            if (!detections.isEmpty()) {
                LLResultTypes.DetectorResult largest = detections.get(0);
                detectedLabel = largest.getClassName();

                double tx = largest.getTargetXDegrees();
                double ty = largest.getTargetYDegrees(); // <--- NEW: Vertical Angle

                // Turn Logic
                turnPower = tx * TURN_GAIN;

                // Drive Logic (Move forward until ty reaches DESIRED_TY)
                // Usually: Far away = ty is 0 or slightly negative. Close = ty is very negative (e.g., -20).
                // Equation: (Current - Target) * Gain
                // Example: Current -5, Target -20. (-5 - -20) = +15. Positive power drives forward.
                drivePower = (ty - DESIRED_TY) * DRIVE_GAIN;

                // Safety Clamp
                if (drivePower > MAX_AUTO_SPEED) drivePower = MAX_AUTO_SPEED;
                if (drivePower < -MAX_AUTO_SPEED) drivePower = -MAX_AUTO_SPEED;
            }
        }

        // 3. Drive Robot (Auto Y, Manual X, Auto Turn)
        // We override the 'y' stick with our calculated drivePower
        driveRobot(drivePower, gamepad1.left_stick_x, turnPower);*/

        // 4. Capture/Exit Logic
        if (gamepad1.dpad_left || gamepad2.dpad_left) {
            if (slots.get(targetSlotIndex).occupied) {
                slots.get(targetSlotIndex).occupied = false;
                slots.get(targetSlotIndex).isClawOpen = true;
                slots.get(targetSlotIndex).color = "Purple";
            }
            else {
                slots.get(targetSlotIndex).occupied = true;
                slots.get(targetSlotIndex).isClawOpen = false;
                slots.get(targetSlotIndex).color = "None";
            }
            updateRevolverServos();
        }
        if (gamepad1.dpad_right || gamepad2.dpad_right) {
            if (slots.get(targetSlotIndex).occupied) {
                slots.get(targetSlotIndex).occupied = false;
                slots.get(targetSlotIndex).isClawOpen = true;
                slots.get(targetSlotIndex).color = "Green";
            }
            else {
                slots.get(targetSlotIndex).occupied = true;
                slots.get(targetSlotIndex).isClawOpen = false;
                slots.get(targetSlotIndex).color = "None";
            }
            updateRevolverServos();
        }

        if (gamepad1.b || gamepad2.b) {
            intakeSpinner.setPower(0.0); // <--- IMPORTANT: Stop intake
            currentState = RobotState.DRIVER_CONTROL;
        }
    }

    // ==========================================================================
    //                         SCORE LOGIC (HuskyLens - Back)
    // ==========================================================================
    private void runScoreLogic() {
        switch (scoringState) {
            case -1:
                if (gamepad1.a || gamepad2.a) {
                    scoringState = 0;
                    actionTimer.resetTimer();
                }
            case 0:
                flywheelL.setPower(SCORING_POWER);
                flywheelR.setPower(SCORING_POWER);
                targetSlotIndex = getSlotWithColor(motif.get(motifIndex));
                revolverServo.setPosition(SCORE_POSITIONS[targetSlotIndex]);
                slots.get(targetSlotIndex).isClawOpen = true;
                if (actionTimer.getElapsedTimeSeconds() > 0.8) {
                    scoringState = 1;
                    actionTimer.resetTimer();
                }
                break;
            case 1:
                kicker.setPosition(KICKER_FIRE);
                if (actionTimer.getElapsedTimeSeconds() > 0.3) {
                    scoringState = 2;
                    actionTimer.resetTimer();
                }
            case 2:
                kicker.setPosition(KICKER_REST);
                if (actionTimer.getElapsedTimeSeconds() > 1.2) {
                    scoringState = -1;

                    actionTimer.resetTimer();

                    slots.get(targetSlotIndex).occupied = false;
                    slots.get(targetSlotIndex).isClawOpen = true;
                    slots.get(targetSlotIndex).color = "None";
                    motifIndex++;
                    motifIndex = motifIndex % motif.size();
                }
                break;
        }
        if (gamepad1.b) {
            scoringState = -1;
            currentState = RobotState.DRIVER_CONTROL;
            flywheelL.setPower(0);
            flywheelR.setPower(0);
        }
    }

}