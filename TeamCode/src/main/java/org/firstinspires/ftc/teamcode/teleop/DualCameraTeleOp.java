package org.firstinspires.ftc.teamcode.teleop;

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
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@TeleOp(name="Dual-Camera Motif TeleOp v2", group="Competition")
public class DualCameraTeleOp extends LinearOpMode {
    // --- HARDWARE ---
    private DcMotor lf, lb, rf, rb;
    private DcMotor flywheelL, flywheelR;
    private Servo revolverServo;
    private Servo claw1, claw2, claw3;
    private Servo kicker;
    private CRServo intakeSpinner; // <--- NEW: Continuous Rotation Intake Servo
    private IMU imu;

    // CAMERAS
    private Limelight3A limelight;   // FRONT: Intake Alignment
    private HuskyLens huskyLens;     // BACK:  Score Alignment

    // --- CONSTANTS ---
    private final double[] INTAKE_POSITIONS = {0.62, 0.245, 1.0};
    private final double[] SCORE_POSITIONS  = {0.075, 0.805, 0.45};
    private final double CLAW_OPEN = 0.03;
    private final double CLAW_CLOSE = 0.17;

    private final double KICKER_REST = 0.75;
    private final double KICKER_FIRE = 0.45;

    private final double SCORING_POWER = -1.0;

    // --- LIMELIGHT DRIVE CONSTANTS (NEW) ---
    // Adjust DESIRED_TY based on how close you want to be to the ball.
    // If the camera is angled down, -20.0 is usually very close, 0.0 is far.
    private final double DESIRED_TY = -18.0;
    private final double DRIVE_GAIN = 0.03;  // Speed multiplier for distance
    private final double TURN_GAIN  = 0.02;  // Speed multiplier for turning
    private final double MAX_AUTO_SPEED = 0.5; // Safety cap

    // --- STATE MACHINE ---
    private enum RobotState {
        DRIVER_CONTROL,
        AUTO_ALIGN_INTAKE, // Uses Limelight (Front)
        AUTO_ALIGN_SCORE   // Uses HuskyLens (Back)
    }
    private RobotState currentState = RobotState.DRIVER_CONTROL;

    // --- GAME LOGIC ---
    private List<List<String>> motifs = new ArrayList<>(Arrays.asList(
            new ArrayList<>(Arrays.asList("Green", "Purple", "Purple")),
            new ArrayList<>(Arrays.asList("Purple", "Green", "Purple")),
            new ArrayList<>(Arrays.asList("Purple", "Purple", "Green"))
    ));
    private List<String> motif = motifs.get(0);
    private int motifIndex = 0;

    private List<IntakeSlot> slots = new ArrayList<>();
    private int targetSlotIndex = -1;

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
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.1;
            double rx = gamepad1.right_stick_x;

            // 2. STATE HANDLING
            switch (currentState) {
                case DRIVER_CONTROL:
                    // Ensure intake is off during normal driving
                    intakeSpinner.setPower(0.0);

                    driveFieldCentric(y, x, rx);

                    if (gamepad1.right_bumper || gamepad2.right_bumper) {
                        targetSlotIndex = getNextEmptySlot();
                        if (targetSlotIndex != -1) {
                            driveDirection = 1.0;
                            currentState = RobotState.AUTO_ALIGN_INTAKE;
                        }
                    }

                    if (gamepad1.dpad_left || gamepad2.dpad_left ||
                        gamepad1.dpad_right || gamepad2.dpad_right) {
                        String requiredColor;
                        if (gamepad1.dpad_left || gamepad2.dpad_left) requiredColor = "Purple";
                        else requiredColor = "Green";

                        targetSlotIndex = getSlotWithColor(requiredColor);

                        if (targetSlotIndex != -1) {
                            driveDirection = -1.0;
                            currentState = RobotState.AUTO_ALIGN_SCORE;
                        } else {
                            telemetry.addData("Alert", "No " + requiredColor + " found!");
                        }
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
        intakeSpinner.setPower(1.0); // <--- NEW: Spins 'in'
        // 2. Servo Setup
        if (targetSlotIndex != -1)
            revolverServo.setPosition(INTAKE_POSITIONS[targetSlotIndex]);
        slots.get(targetSlotIndex).isClawOpen = true;

        LLResult result = limelight.getLatestResult();
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
        driveRobot(drivePower, gamepad1.left_stick_x, turnPower);

        // 4. Capture/Exit Logic
        if (gamepad1.a || gamepad2.a) {
            slots.get(targetSlotIndex).occupied = true;
            // Simple color logic based on label
            if (detectedLabel.toLowerCase().contains("green")) {
                slots.get(targetSlotIndex).color = "Green";
            } else {
                slots.get(targetSlotIndex).color = "Purple";
            }

            slots.get(targetSlotIndex).isClawOpen = false;
            intakeSpinner.setPower(0.0); // <--- IMPORTANT: Stop intake
            currentState = RobotState.DRIVER_CONTROL;
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
        revolverServo.setPosition(SCORE_POSITIONS[targetSlotIndex]);

        flywheelL.setPower(SCORING_POWER);
        flywheelR.setPower(SCORING_POWER);

        HuskyLens.Block[] blocks = huskyLens.blocks();
        double turnPower = 0;

        if (blocks.length > 0) {
            int targetX = blocks[0].x;
            double error = 120 - targetX;
            turnPower = error * 0.005;
        }

        driveRobot(gamepad1.left_stick_y, gamepad1.left_stick_x, turnPower);

        if (gamepad1.a || gamepad2.a) {
            slots.get(targetSlotIndex).isClawOpen = true;
            updateRevolverServos();

            sleep(800);
            kicker.setPosition(KICKER_FIRE);
            sleep(300);

            kicker.setPosition(KICKER_REST);
            sleep(1200);

            slots.get(targetSlotIndex).occupied = false;
            slots.get(targetSlotIndex).color = "None";

            motifIndex++;
            motifIndex = motifIndex % motif.size();

            flywheelL.setPower(0);
            flywheelR.setPower(0);
            driveDirection = 1.0;
            currentState = RobotState.DRIVER_CONTROL;
        }

        if (gamepad1.b || gamepad2.b) {
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
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
        rotX = rotX * 1.1;

        if (driveDirection < 0) {
            rotX = -rotX;
            rotY = -rotY;
        }

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        lf.setPower((rotY + rotX + rx) / denominator);
        lb.setPower((rotY - rotX + rx) / denominator);
        rf.setPower((rotY - rotX - rx) / denominator);
        rb.setPower((rotY + rotX - rx) / denominator);
    }

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

        flywheelL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flywheelR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        revolverServo = hardwareMap.get(Servo.class, "revolver");
        claw1 = hardwareMap.get(Servo.class, "slot1");
        claw2 = hardwareMap.get(Servo.class, "slot2");
        claw3 = hardwareMap.get(Servo.class, "slot3");

        kicker = hardwareMap.get(Servo.class, "kicker");
        kicker.setPosition(KICKER_REST);

        // --- INTAKE SPINNER ---
        intakeSpinner = hardwareMap.get(CRServo.class, "intake");
        intakeSpinner.setPower(0); // Ensure off at start

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);
    }
}