package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.teleop.PrototypeTeleop;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@Disabled
@Autonomous(name = "Rear double score", group = "auto")
public class AutoExample extends PrototypeTeleop {
    Follower follower;
    // Pedropathing variables;

    public Timer pathTimer, actionTimer, opmodeTimer;

    public int pathState;
    public int scoringState;
    // --------------------------------
    //                          red
    // idk
    public Pose currentPose;
    public Path toLaunchZone;


    // ==========================================================================
    //                         INTAKE LOGIC (Limelight - Front)
    // ==========================================================================
    public void runIntakeLogic() {
        /*
        // 1. Spin Intake Active
        intakeMotor.setPower(1.0); // <--- NEW: Spins 'in'

        // 2. Servo Setup

        LLResult result = limelight.getLatestResult();
        double turnPower = 0;
        double drivePower = 0; // <--- NEW: Forward/Back Power
        String detectedLabel = "Unknown";

        if (result != null && result.isValid()) {
            List < LLResultTypes.DetectorResult > detections = result.getDetectorResults();
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
        //driveRobot(drivePower, gamepad1.left_stick_x, turnPower);

        // 4. Capture/Exit Logic
        if (gamepad1.a) {
            slots.get(targetSlotIndex).occupied = true;
            // Simple color logic based on label
            if (detectedLabel.toLowerCase().contains("green")) {
                slots.get(targetSlotIndex).color = "Green";
            } else {
                slots.get(targetSlotIndex).color = "Purple";
            }

            slots.get(targetSlotIndex).isClawOpen = false;
            intakeSpinner.setPower(0.0); // <--- IMPORTANT: Stop intake
        }
        
         */
    }

    // ==========================================================================
    //                         SCORE LOGIC (HuskyLens - Back)
    public void runScoreLogic(boolean highSpeed) {

        switch (scoringState) {
            case 0:
                actionTimer.resetTimer();
                scoringState = 1;

                break;
            case 1:
                leftFlywheel.setPower(SHOOTER_VELOCITY);
                rightFlywheel.setPower(SHOOTER_VELOCITY);
                if (actionTimer.getElapsedTime() > 2000) {
                    scoringState = 2;
                    actionTimer.resetTimer();
                }

                break;
            case 2:
                middleMotor.setPower(1.0);
                intakeMotor.setPower(1.0);
                if (actionTimer.getElapsedTime() > 8000) {
                    scoringState = -1;
                }

                break;

        }
    }
}

