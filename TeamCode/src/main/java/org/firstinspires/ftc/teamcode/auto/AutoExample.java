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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@Autonomous(name = "Rear double score", group = "auto")
public class AutoExample extends LinearOpMode {
    Follower follower;
    public DcMotor lf, lb, rf, rb;
    public DcMotor flywheelL, flywheelR;
    public Servo revolverServo;
    public Servo claw1, claw2, claw3;
    public Servo kicker;
    public CRServo intakeSpinner; // <--- NEW: Continuous Rotation Intake Servo
    public IMU imu;

    // CAMERAS
    public Limelight3A limelight; // FRONT: Intake Alignment
    public HuskyLens huskyLens; // BACK:  Score Alignment

    // --- CONSTANTS ---
    public final double[] INTAKE_POSITIONS = {
            0.62,
            0.245,
            1.0
    };
    public final double[] SCORE_POSITIONS = {
            0.075,
            0.805,
            0.45
    };

    public final double KICKER_REST = 0.75;
    public final double KICKER_FIRE = 0.54;

    public final double SCORING_POWER_HIGH = -0.6;
    public final double SCORING_POWER_LOW = -0.6;

    // --- LIMELIGHT DRIVE CONSTANTS (NEW) ---
    // Adjust DESIRED_TY based on how close you want to be to the ball.
    // If the camera is angled down, -20.0 is usually very close, 0.0 is far.
    public final double DESIRED_TY = -18.0;
    public final double DRIVE_GAIN = 0.03; // Speed multiplier for distance
    public final double TURN_GAIN = 0.02; // Speed multiplier for turning
    public final double MAX_AUTO_SPEED = 0.5; // Safety cap
    // --- GAME LOGIC ---
    public List < String > motif = new ArrayList < > (Arrays.asList("Purple", "Purple", "Green"));
    public int motifIndex = 0;
    public List < IntakeSlot > slots = new ArrayList < > ();
    public int targetSlotIndex = -1;
    public double driveDirection = 1.0;
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
        // 1. Spin Intake Active
        intakeSpinner.setPower(1.0); // <--- NEW: Spins 'in'

        // 2. Servo Setup
        revolverServo.setPosition(INTAKE_POSITIONS[targetSlotIndex]);
        slots.get(targetSlotIndex).isClawOpen = true;

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
    }

    // ==========================================================================
    //                         SCORE LOGIC (HuskyLens - Back)
    public void runScoreLogic(boolean highSpeed) {
        switch (scoringState) {
            case 0:
                flywheelL.setPower(highSpeed ? SCORING_POWER_HIGH : SCORING_POWER_LOW);
                flywheelR.setPower(highSpeed ? SCORING_POWER_HIGH : SCORING_POWER_LOW);
                targetSlotIndex = getSlotWithColor(motif.get(motifIndex));
                if (targetSlotIndex < 0) {
                    scoringState = -1;
                    flywheelL.setPower(0);
                    flywheelR.setPower(0);
                }
                else {
                    revolverServo.setPosition(SCORE_POSITIONS[targetSlotIndex]);
                    slots.get(targetSlotIndex).isClawOpen = true;
                    updateRevolverServos();
                    if (actionTimer.getElapsedTimeSeconds() > 1.2) {
                        scoringState = 1;
                        actionTimer.resetTimer();
                    }
                }
                break;
            case 1:
                kicker.setPosition(KICKER_FIRE);
                if (actionTimer.getElapsedTimeSeconds() > 0.5) {
                    scoringState = 2;
                    actionTimer.resetTimer();
                }
                break;
            case 2:
                kicker.setPosition(KICKER_REST);
                if (actionTimer.getElapsedTimeSeconds() > 0.8) {
                    scoringState = 0;
                    actionTimer.resetTimer();

                    slots.get(targetSlotIndex).occupied = false;
                    slots.get(targetSlotIndex).isClawOpen = false;
                    updateRevolverServos();
                    slots.get(targetSlotIndex).color = "None";
                    motifIndex++;
                    motifIndex = motifIndex % motif.size();
                }
                break;
        }
    }
    public void startIntake() {
        revolverServo.setPosition(INTAKE_POSITIONS[targetSlotIndex]);
        slots.get(targetSlotIndex).isClawOpen = true;

    }

    // ==========================================================================
    // ==========================================================================
    //                             HELPER CLASSES
    // ==========================================================================

    public void initLogic() {
        slots.add(new IntakeSlot(1, claw1));
        slots.add(new IntakeSlot(2, claw2));
        slots.add(new IntakeSlot(3, claw3));

        slots.get(0).occupied = true;
        slots.get(0).color = "Purple";
        slots.get(1).occupied = true;
        slots.get(1).color = "Purple";
        slots.get(2).occupied = true;
        slots.get(2).color = "Green";
    }

    public int getNextEmptySlot() {
        for (int i = 0; i < 3; i++) {
            if (!slots.get(i).occupied) return i;
        }
        return -1;
    }

    public int getSlotWithColor(String neededColor) {
        for (int i = 0; i < 3; i++) {
            if (slots.get(i).occupied && slots.get(i).color.equalsIgnoreCase(neededColor)) {
                return i;
            }
        }
        return -1;
    }
    public boolean allSlotsEmpty() {
        for (int i = 0; i < 3; i++)
            if (slots.get(i).occupied)
                return false;
        return true;
    }
    public void updateRevolverServos() {
        for (IntakeSlot slot: slots) slot.updateServo();
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
    /**
     * These change the states of the paths and actions. It will also reset the timers of the individual switches
     **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }
    public void runOpMode() {
        
        
    }
    class IntakeSlot {
        public final double CLAW_OPEN = 0.0;
        public final double CLAW_CLOSE = 0.23;
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
    public Pose getRobotPoseFromCamera() {
        //Fill this out to get the robot Pose from the camera's output (apply any filters if you need to using follower.getPose() for fusion)
        //Pedro Pathing has built-in KalmanFilter and LowPassFilter classes you can use for this
        //Use this to convert standard FTC coordinates to standard Pedro Pathing coordinates
        //return new follower.getPose();
        return new Pose(0, 0, 0, FTCCoordinates.INSTANCE).getAsCoordinateSystem(PedroCoordinates.INSTANCE);
    }
}
