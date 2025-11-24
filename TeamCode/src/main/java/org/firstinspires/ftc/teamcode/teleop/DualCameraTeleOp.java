package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@TeleOp(name="Dual-Camera Motif TeleOp", group="Competition")
public class DualCameraTeleOp extends LinearOpMode {

    // --- HARDWARE ---
    private DcMotor lf, lb, rf, rb;
    private DcMotor flywheelL, flywheelR;
    private Servo revolverServo;
    private Servo claw1, claw2, claw3;
    private IMU imu;

    // CAMERAS
    private Limelight3A limelight;   // FRONT: Intake Alignment
    private HuskyLens huskyLens;     // BACK:  Score Alignment

    // --- CONSTANTS ---
    // Update these specific values with your servo programmer!
    private final double[] INTAKE_POSITIONS = {0.62, 0.245, 1.0};
    private final double[] SCORE_POSITIONS  = {0.075, 0.805, 0.45};
    private final double CLAW_OPEN = 0.3;
    private final double CLAW_CLOSE = 0.6;

    // --- STATE MACHINE ---
    private enum RobotState {
        DRIVER_CONTROL,
        AUTO_ALIGN_INTAKE, // Uses Limelight (Front)
        AUTO_ALIGN_SCORE   // Uses HuskyLens (Back)
    }
    private RobotState currentState = RobotState.DRIVER_CONTROL;

    // --- GAME LOGIC ---
    // The Motif: The pattern we MUST score in.
    private List<String> motif = new ArrayList<>(Arrays.asList("Purple", "Purple", "Green"));
    private int motifIndex = 0;

    // Slots management
    private List<IntakeSlot> slots = new ArrayList<>();
    private int targetSlotIndex = -1;

    // Driving direction multiplier (1 for Forward, -1 for Backward)
    private double driveDirection = 1.0;

    @Override
    public void runOpMode() {
        initHardware();
        initLogic();

        telemetry.addData("Status", "Initialized. Preloaded: P, P, G");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // 1. INPUTS
            // Standard Field Centric Inputs
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.1;
            double rx = gamepad1.right_stick_x;

            // 2. STATE HANDLING
            switch (currentState) {
                case DRIVER_CONTROL:
                    // Standard Drive
                    driveFieldCentric(y, x, rx);

                    // Button: INTAKE (Right Bumper)
                    if (gamepad1.right_bumper) {
                        targetSlotIndex = getNextEmptySlot();
                        if (targetSlotIndex != -1) {
                            driveDirection = 1.0; // Drive Forward
                            currentState = RobotState.AUTO_ALIGN_INTAKE;
                        }
                    }

                    // Button: SCORE (Left Bumper)
                    if (gamepad1.left_bumper) {
                        // Find the slot that matches the current Motif color
                        String requiredColor = motif.get(motifIndex);
                        targetSlotIndex = getSlotWithColor(requiredColor);

                        if (targetSlotIndex != -1) {
                            driveDirection = -1.0; // Drive Backward (Camera is on back)
                            currentState = RobotState.AUTO_ALIGN_SCORE;
                        } else {
                            telemetry.addData("Alert", "No " + requiredColor + " found!");
                        }
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
        // 1. Servo to Position
        revolverServo.setPosition(INTAKE_POSITIONS[targetSlotIndex]);
        slots.get(targetSlotIndex).isClawOpen = true;

        // 2. Limelight Tracking (TensorFlow / Neural Detector)
        LLResult result = limelight.getLatestResult();
        double turnPower = 0;
        String detectedLabel = "Unknown";

        if (result != null && result.isValid()) {
            // Get the list of detections (e.g., "Purple", "Green")
            List<LLResultTypes.DetectorResult> detections = result.getDetectorResults();

            if (!detections.isEmpty()) {
                // Track the largest object
                LLResultTypes.DetectorResult largest = detections.get(0);
                detectedLabel = largest.getClassName(); // Requires correct pipeline setup!

                // PID Turn
                double tx = largest.getTargetXDegrees();
                turnPower = -tx * 0.03;
            }
        }

        // 3. Drive (Driver controls movement, Robot controls Turn)
        driveRobot(gamepad1.left_stick_y, gamepad1.left_stick_x, turnPower);

        // 4. Confirm Intake (Press A)
        if (gamepad1.a) {
            slots.get(targetSlotIndex).occupied = true;

            // Save the detected color (or default to Purple if detection fails)
            if (detectedLabel.toLowerCase().contains("green")) {
                slots.get(targetSlotIndex).color = "Green";
            } else {
                slots.get(targetSlotIndex).color = "Purple";
            }

            slots.get(targetSlotIndex).isClawOpen = false; // Close
            currentState = RobotState.DRIVER_CONTROL;
        }

        if (gamepad1.b) currentState = RobotState.DRIVER_CONTROL;
    }

    // ==========================================================================
    //                         SCORE LOGIC (HuskyLens - Back)
    // ==========================================================================
    private void runScoreLogic() {
        // 1. Servo to Position
        revolverServo.setPosition(SCORE_POSITIONS[targetSlotIndex]);

        // 2. Spin Flywheels
        flywheelL.setPower(1.0);
        flywheelR.setPower(1.0);

        // 3. HuskyLens Tracking (AprilTag or Color)
        HuskyLens.Block[] blocks = huskyLens.blocks();
        double turnPower = 0;

        if (blocks.length > 0) {
            // HuskyLens resolution is 320x240. Center is 160.
            // If block is at 100 (left of screen), robot (back) needs to turn Left.
            // Since camera faces BACK, a Left turn for the Camera is a Left turn for the Robot.

            int targetX = blocks[0].x; // X coordinate of the target
            double error = 160 - targetX; // Positive if target is to the Left

            // Simple P-Controller
            turnPower = error * 0.005;
        }

        // 4. Drive Logic (Inverted because we are driving backwards)
        // We flip the stick inputs so "Up" on stick moves the robot "Backwards" (towards scoring)
        driveRobot(-gamepad1.left_stick_y, -gamepad1.left_stick_x, turnPower);

        // 5. Shoot (Press A)
        if (gamepad1.a) {
            slots.get(targetSlotIndex).isClawOpen = true;
            sleep(250); // Small wait for release

            // Reset Slot
            slots.get(targetSlotIndex).occupied = false;
            slots.get(targetSlotIndex).color = "None";

            // Next Motif
            motifIndex = (motifIndex + 1) % motif.size();

            // Cleanup
            flywheelL.setPower(0);
            flywheelR.setPower(0);
            driveDirection = 1.0; // Reset to forward driving
            currentState = RobotState.DRIVER_CONTROL;
        }

        if (gamepad1.b) {
            flywheelL.setPower(0);
            flywheelR.setPower(0);
            driveDirection = 1.0;
            currentState = RobotState.DRIVER_CONTROL;
        }
    }

    // ==========================================================================
    //                             HELPER CLASSES
    // ==========================================================================

    class IntakeSlot {
        int id;
        boolean occupied;
        String color;
        boolean isClawOpen;
        Servo clawServo;

        public IntakeSlot(int id, Servo servo) {
            this.id = id;
            this.clawServo = servo;
            this.occupied = false;
            this.color = "None";
            this.isClawOpen = false;
        }

        public void updateServo() {
            clawServo.setPosition(isClawOpen ? CLAW_OPEN : CLAW_CLOSE);
        }
    }

    private void initLogic() {
        slots.add(new IntakeSlot(1, claw1));
        slots.add(new IntakeSlot(2, claw2));
        slots.add(new IntakeSlot(3, claw3));

        // PRELOAD: Green (1), Purple (2), Purple (3)
        slots.get(0).occupied = true; slots.get(0).color = "Green";
        slots.get(1).occupied = true; slots.get(1).color = "Purple";
        slots.get(2).occupied = true; slots.get(2).color = "Purple";
    }

    private int getNextEmptySlot() {
        for (int i = 0; i < 3; i++) {
            if (!slots.get(i).occupied) return i;
        }
        return -1;
    }

    private int getSlotWithColor(String neededColor) {
        for (int i = 0; i < 3; i++) {
            if (slots.get(i).occupied && slots.get(i).color.equalsIgnoreCase(neededColor)) {
                return i;
            }
        }
        return -1;
    }

    private void updateRevolverServos() {
        for (IntakeSlot slot : slots) slot.updateServo();
    }

    // ==========================================================================
    //                             DRIVETRAIN
    // ==========================================================================

    private void driveFieldCentric(double y, double x, double rx) {
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Rotate control vector by -heading
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.1;

        // Apply Drive Direction (flips controls if we are backing up to score)
        // Note: In field centric, "Forward" is always "Away from driver".
        // We usually don't flip field centric controls, but we flip Robot Centric.
        // If you want the robot to feel like the "Back" is now the "Front":
        if (driveDirection < 0) {
            // Optional: Rotate the coordinate system 180 degrees
            rotX = -rotX;
            rotY = -rotY;
        }

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        lf.setPower((rotY + rotX + rx) / denominator);
        lb.setPower((rotY - rotX + rx) / denominator);
        rf.setPower((rotY - rotX - rx) / denominator);
        rb.setPower((rotY + rotX - rx) / denominator);
    }

    // Robot Centric Drive for Auto-Alignment
    private void driveRobot(double y, double x, double rx) {
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        lf.setPower((y + x + rx) / denominator);
        lb.setPower((y - x + rx) / denominator);
        rf.setPower((y - x - rx) / denominator);
        rb.setPower((y + x - rx) / denominator);
    }

    private void initHardware() {
        lf = hardwareMap.get(DcMotor.class, "flDrive");
        lb = hardwareMap.get(DcMotor.class, "rlDrive");
        rf = hardwareMap.get(DcMotor.class, "frDrive");
        rb = hardwareMap.get(DcMotor.class, "rrDrive");

        lf.setDirection(DcMotor.Direction.REVERSE);
        lb.setDirection(DcMotor.Direction.REVERSE);
        rf.setDirection(DcMotor.Direction.FORWARD);
        rb.setDirection(DcMotor.Direction.FORWARD);

        flywheelL = hardwareMap.get(DcMotor.class, "leftShooter");
        flywheelR = hardwareMap.get(DcMotor.class, "rightShooter");
        flywheelR.setDirection(DcMotorSimple.Direction.REVERSE);

        revolverServo = hardwareMap.get(Servo.class, "revolver");
        claw1 = hardwareMap.get(Servo.class, "slot1");
        claw2 = hardwareMap.get(Servo.class, "slot2");
        claw3 = hardwareMap.get(Servo.class, "slot3");

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);

        // LIMELIGHT INIT
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0); // Ensure this pipeline has your Neural Detector
        limelight.start();

        // HUSKYLENS INIT
        huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION); // Or COLOR_RECOGNITION
    }
}