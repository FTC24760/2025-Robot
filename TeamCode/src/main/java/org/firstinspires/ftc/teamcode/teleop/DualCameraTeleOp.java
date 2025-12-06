package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime; // <--- IMPORT ADDED
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@TeleOp(name="Dual-Camera Motif TeleOp v5", group="Competition")
public class DualCameraTeleOp extends LinearOpMode {
    // --- HARDWARE ---
    public DcMotor lf, lb, rf, rb;
    public DcMotor flywheelL, flywheelR;
    public Servo revolverServo;
    public Servo claw1, claw2, claw3;
    public Servo kicker;
    public CRServo intakeSpinner;
    public IMU imu;

    // CAMERAS
    public Limelight3A limelight;
    public HuskyLens huskyLens;

    // --- TIMERS ---
    public ElapsedTime batchTimer = new ElapsedTime(); // <--- NEW TIMER

    // --- CONSTANTS ---
    public final double[] INTAKE_POSITIONS = {0.62, 0.245, 1.0};
    public final double[] SCORE_POSITIONS  = {0.075, 0.805, 0.45};
    public final double CLAW_OPEN = 0.03;
    public final double CLAW_CLOSE = 0.23;

    public final double KICKER_REST = 0.75;
    public final double KICKER_FIRE = 0.45;
    public final double SCORING_POWER = -0.6;

    // --- LIMELIGHT DRIVE CONSTANTS ---
    public final double DESIRED_TY = -18.0;
    public final double DRIVE_GAIN = 0.03;
    public final double TURN_GAIN  = 0.02;
    public final double MAX_AUTO_SPEED = 0.5;

    // --- BATCH SETTINGS ---
    // The robot will drive for this many milliseconds before snapping the claw
    public final double BATCH_CYCLE_TIME = 1500; // 1.5 Seconds

    // --- STATE MACHINE ---
    public enum RobotState {
        DRIVER_CONTROL,
        AUTO_ALIGN_INTAKE,
        AUTO_BATCH_INTAKE,
        AUTO_ALIGN_SCORE,
        AUTO_RAPID_FIRE
    }
    public RobotState currentState = RobotState.DRIVER_CONTROL;

    // --- GAME LOGIC ---
    public List<List<String>> motifs = new ArrayList<>(Arrays.asList(
            new ArrayList<>(Arrays.asList("Green", "Purple", "Purple")),
            new ArrayList<>(Arrays.asList("Purple", "Green", "Purple")),
            new ArrayList<>(Arrays.asList("Purple", "Purple", "Green"))
    ));
    public List<String> motif = motifs.get(0);
    public int motifIndex = 0;

    public List<IntakeSlot> slots = new ArrayList<>();
    public int targetSlotIndex = -1;
    public double driveDirection = 1.0;

    @Override
    public void runOpMode() {
        initHardware();
        initLogic();

        telemetry.addData("Status", "Initialized. Batch Timer Ready.");
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
                    intakeSpinner.setPower(0.0);
                    driveFieldCentric(y, x, rx);

                    // Single Intake
                    if (gamepad1.right_bumper || gamepad2.right_bumper) {
                        targetSlotIndex = getNextEmptySlot();
                        if (targetSlotIndex != -1) {
                            driveDirection = 1.0;
                            currentState = RobotState.AUTO_ALIGN_INTAKE;
                        }
                    }

                    // --- BATCH INTAKE TRIGGER ---
                    if (gamepad1.dpad_up) {
                        if (getNextEmptySlot() != -1) {
                            driveDirection = 1.0;
                            targetSlotIndex = -1; // Reset to force setup
                            batchTimer.reset();   // <--- RESET TIMER ON START
                            currentState = RobotState.AUTO_BATCH_INTAKE;
                        } else {
                            telemetry.addData("Alert", "Full!");
                        }
                    }

                    // Scoring
                    if (gamepad1.dpad_left || gamepad2.dpad_left ||
                            gamepad1.dpad_right || gamepad2.dpad_right) {
                        String requiredColor = (gamepad1.dpad_left || gamepad2.dpad_left) ? "Purple" : "Green";
                        targetSlotIndex = getSlotWithColor(requiredColor);
                        if (targetSlotIndex != -1) {
                            driveDirection = -1.0;
                            currentState = RobotState.AUTO_ALIGN_SCORE;
                        }
                    }

                    if (gamepad1.left_bumper) currentState = RobotState.AUTO_RAPID_FIRE;
                    if (gamepad1.right_trigger > 0.3) imu.resetYaw();
                    break;

                case AUTO_ALIGN_INTAKE:
                    runIntakeLogic();
                    break;

                case AUTO_BATCH_INTAKE:
                    runBatchIntakeTimerLogic(); // <--- NEW TIMER FUNCTION
                    break;

                case AUTO_ALIGN_SCORE:
                    runScoreLogic();
                    break;

                case AUTO_RAPID_FIRE:
                    runRapidFireSequence();
                    break;
            }

            // 3. HARDWARE UPDATES
            updateRevolverServos();

            // 4. TELEMETRY
            telemetry.addData("Mode", currentState);
            telemetry.addData("Batch Timer", batchTimer.milliseconds());
            telemetry.addData("Slots", slots.get(0).color + ", " + slots.get(1).color + ", " + slots.get(2).color);
            telemetry.update();
        }
    }

    // ==========================================================================
    //                  BATCH INTAKE LOGIC (TIMER BASED)
    // ==========================================================================
    public void runBatchIntakeTimerLogic() {
        // 1. SETUP: Find next slot if needed
        if (targetSlotIndex == -1 || slots.get(targetSlotIndex).occupied) {
            targetSlotIndex = getNextEmptySlot();

            // Exit if full
            if (targetSlotIndex == -1) {
                intakeSpinner.setPower(0);
                driveRobot(0,0,0);
                currentState = RobotState.DRIVER_CONTROL;
                return;
            }

            // Prepare mechanism
            revolverServo.setPosition(INTAKE_POSITIONS[targetSlotIndex]);
            slots.get(targetSlotIndex).isClawOpen = true;
            updateRevolverServos();

            // Restart the timer for this specific ball attempt
            batchTimer.reset();
        }

        // 2. DRIVE LOGIC (Limelight alignment)
        intakeSpinner.setPower(1.0);

        LLResult result = limelight.getLatestResult();
        double drivePower = -gamepad1.left_stick_y;
        double turnPower  = gamepad1.right_stick_x;
        double strafePower = gamepad1.left_stick_x;
        String detectedLabel = "Unknown";

        if (result != null && result.isValid()) {
            List<LLResultTypes.DetectorResult> detections = result.getDetectorResults();
            if (!detections.isEmpty()) {
                LLResultTypes.DetectorResult largest = detections.get(0);
                detectedLabel = largest.getClassName();

                double tx = largest.getTargetXDegrees();
                double ty = largest.getTargetYDegrees();

                turnPower = tx * TURN_GAIN;
                drivePower = (ty - DESIRED_TY) * DRIVE_GAIN; // Keeps driving to target

                if (drivePower > MAX_AUTO_SPEED) drivePower = MAX_AUTO_SPEED;
                if (drivePower < -MAX_AUTO_SPEED) drivePower = -MAX_AUTO_SPEED;
            }
        }

        // 3. CHECK TIMER
        // If timer exceeds limit, we assume we grabbed it (or missed and need to move on)
        if (batchTimer.milliseconds() > BATCH_CYCLE_TIME) {
            // A. STOP
            driveRobot(0, 0, 0);

            // B. CLAMP
            slots.get(targetSlotIndex).isClawOpen = false;
            updateRevolverServos();

            // C. LOG DATA (Guess color based on last vision data)
            slots.get(targetSlotIndex).occupied = true;
            if (detectedLabel.toLowerCase().contains("green")) {
                slots.get(targetSlotIndex).color = "Green";
            } else {
                slots.get(targetSlotIndex).color = "Purple";
            }

            sleep(200); // Tiny pause to ensure physical clamp

            // D. RESET TARGET
            // Setting this to -1 forces step 1 (Setup) to run again in the next loop
            // which will find the NEXT slot and reset the timer.
            targetSlotIndex = -1;

        } else {
            // Timer running: Keep Driving
            driveRobot(drivePower, strafePower, turnPower);
        }

        // Cancel
        if (gamepad1.b || gamepad2.b) {
            intakeSpinner.setPower(0);
            currentState = RobotState.DRIVER_CONTROL;
        }
    }

    // ==========================================================================
    //                  EXISTING LOGIC BELOW
    // ==========================================================================

    private void runIntakeLogic() {
        intakeSpinner.setPower(1.0);
        if (targetSlotIndex != -1) {
            revolverServo.setPosition(INTAKE_POSITIONS[targetSlotIndex]);
            slots.get(targetSlotIndex).isClawOpen = true;
        }

        LLResult result = limelight.getLatestResult();
        double drivePower = -gamepad1.left_stick_y;
        double turnPower  = gamepad1.right_stick_x;
        double strafePower = gamepad1.left_stick_x;
        String detectedLabel = "Unknown";

        if (result != null && result.isValid()) {
            List<LLResultTypes.DetectorResult> detections = result.getDetectorResults();
            if (!detections.isEmpty()) {
                LLResultTypes.DetectorResult largest = detections.get(0);
                detectedLabel = largest.getClassName();
                double tx = largest.getTargetXDegrees();
                double ty = largest.getTargetYDegrees();

                turnPower = tx * TURN_GAIN;
                drivePower = (ty - DESIRED_TY) * DRIVE_GAIN;

                if (drivePower > MAX_AUTO_SPEED) drivePower = MAX_AUTO_SPEED;
                if (drivePower < -MAX_AUTO_SPEED) drivePower = -MAX_AUTO_SPEED;
            }
        }

        driveRobot(drivePower, strafePower, turnPower);

        if (gamepad1.a || gamepad2.a) {
            slots.get(targetSlotIndex).occupied = true;
            if (detectedLabel.toLowerCase().contains("green")) slots.get(targetSlotIndex).color = "Green";
            else slots.get(targetSlotIndex).color = "Purple";
            intakeSpinner.setPower(0.0);
            sleep(300);

            slots.get(targetSlotIndex).isClawOpen = false;
            updateRevolverServos();
            currentState = RobotState.DRIVER_CONTROL;
        }
        if (gamepad1.b || gamepad2.b) {
            intakeSpinner.setPower(0.0);
            currentState = RobotState.DRIVER_CONTROL;
        }
    }

    private void runScoreLogic() {
        revolverServo.setPosition(SCORE_POSITIONS[targetSlotIndex]);
        flywheelL.setPower(SCORING_POWER);
        flywheelR.setPower(SCORING_POWER);

        HuskyLens.Block[] blocks = huskyLens.blocks();
        double turnPower = 0;
        if (blocks.length > 0) {
            int targetX = blocks[0].x;
            //double error = 160 - targetX;
            //turnPower = error * 0.005;
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
            slots.get(targetSlotIndex).isClawOpen = false;
            updateRevolverServos();
            slots.get(targetSlotIndex).occupied = false;
            slots.get(targetSlotIndex).color = "None";
            motifIndex = (motifIndex + 1) % motif.size();
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

    public void runRapidFireSequence() {
        flywheelL.setPower(SCORING_POWER);
        flywheelR.setPower(SCORING_POWER);
        telemetry.addData("Mode", "RAPID FIRE");
        telemetry.update();
        sleep(1500);

        for (int i = 0; i < 3; i++) {
            if (!opModeIsActive()) break;
            revolverServo.setPosition(SCORE_POSITIONS[i]);
            sleep(800);
            slots.get(i).isClawOpen = true;
            updateRevolverServos();
            sleep(400);
            kicker.setPosition(KICKER_FIRE);
            sleep(300);
            kicker.setPosition(KICKER_REST);
            sleep(300);
            slots.get(i).occupied = false;
            slots.get(i).color = "None";
            slots.get(i).isClawOpen = false;
            updateRevolverServos();
        }
        flywheelL.setPower(0);
        flywheelR.setPower(0);
        driveDirection = 1.0;
        currentState = RobotState.DRIVER_CONTROL;
    }

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

    public void initLogic() {
        slots.add(new IntakeSlot(1, claw1));
        slots.add(new IntakeSlot(2, claw2));
        slots.add(new IntakeSlot(3, claw3));

        slots.get(0).occupied = true; slots.get(0).color = "Green";
        slots.get(1).occupied = true; slots.get(1).color = "Purple";
        slots.get(2).occupied = true; slots.get(2).color = "Purple";
    }

    public int getNextEmptySlot() {
        for (int i = 0; i < 3; i++) if (!slots.get(i).occupied) return i;
        return -1;
    }

    public int getSlotWithColor(String neededColor) {
        for (int i = 0; i < 3; i++) if (slots.get(i).occupied && slots.get(i).color.equalsIgnoreCase(neededColor)) return i;
        return -1;
    }

    public void updateRevolverServos() {
        for (IntakeSlot slot : slots) slot.updateServo();
    }

    public void driveFieldCentric(double y, double x, double rx) {
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
        rotX = rotX * 1.1;
        if (driveDirection < 0) { rotX = -rotX; rotY = -rotY; }
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        lf.setPower((rotY + rotX + rx) / denominator);
        lb.setPower((rotY - rotX + rx) / denominator);
        rf.setPower((rotY - rotX - rx) / denominator);
        rb.setPower((rotY + rotX - rx) / denominator);
    }

    public void driveRobot(double y, double x, double rx) {
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        lf.setPower((y + x + rx) / denominator);
        lb.setPower((y - x + rx) / denominator);
        rf.setPower((y - x - rx) / denominator);
        rb.setPower((y + x - rx) / denominator);
    }

    public void initHardware() {
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
        intakeSpinner = hardwareMap.get(CRServo.class, "intake");
        intakeSpinner.setPower(0);

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