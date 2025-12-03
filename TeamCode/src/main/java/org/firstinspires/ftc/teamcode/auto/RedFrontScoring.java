package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
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

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@Autonomous(name = "Red Front", group = "Auto")
public class RedFrontScoring extends LinearOpMode {
    private DcMotor lf, lb, rf, rb;
    private DcMotor flywheelL, flywheelR;
    private Servo revolverServo;
    private Servo claw1, claw2, claw3;
    private Servo kicker;
    private CRServo intakeSpinner; // <--- NEW: Continuous Rotation Intake Servo
    private IMU imu;

    // CAMERAS
    private Limelight3A limelight; // FRONT: Intake Alignment
    private HuskyLens huskyLens; // BACK:  Score Alignment

    // --- CONSTANTS ---
    private final double[] INTAKE_POSITIONS = {
            0.62,
            0.245,
            1.0
    };
    private final double[] SCORE_POSITIONS = {
            0.075,
            0.805,
            0.45
    };

    private final double KICKER_REST = 0.75;
    private final double KICKER_FIRE = 0.54;

    private final double SCORING_POWER = -0.67;

    // --- LIMELIGHT DRIVE CONSTANTS (NEW) ---
    // Adjust DESIRED_TY based on how close you want to be to the ball.
    // If the camera is angled down, -20.0 is usually very close, 0.0 is far.
    private final double DESIRED_TY = -18.0;
    private final double DRIVE_GAIN = 0.03; // Speed multiplier for distance
    private final double TURN_GAIN = 0.02; // Speed multiplier for turning
    private final double MAX_AUTO_SPEED = 0.5; // Safety cap
    // --- GAME LOGIC ---
    private List < String > motif = new ArrayList < > (Arrays.asList("Purple", "Purple", "Green"));
    private int motifIndex = 0;
    private List < IntakeSlot > slots = new ArrayList < > ();
    private int targetSlotIndex = -1;
    private double driveDirection = 1.0;
    // Pedropathing variables;
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState;
    private int scoringState;
    // --------------------------------
    //                          red
    // idk
    private final Pose startPose = new Pose(96, 9, Math.toRadians(90));
    private final Pose scorePose = new Pose(84, 84, Math.toRadians(45));
    private final Pose finalPose = new Pose(108, 10, Math.toRadians(270));
    private Pose currentPose;
    private Path toLaunchZone;
    public void buildPaths() {
        toLaunchZone = new Path(new BezierLine(scorePose, finalPose));
    }
    public Pose getRobotPoseFromCamera() {
        //Fill this out to get the robot Pose from the camera's output (apply any filters if you need to using follower.getPose() for fusion)
        //Pedro Pathing has built-in KalmanFilter and LowPassFilter classes you can use for this
        //Use this to convert standard FTC coordinates to standard Pedro Pathing coordinates
        return new Pose(0, 0, 0, FTCCoordinates.INSTANCE).getAsCoordinateSystem(PedroCoordinates.INSTANCE);
    }

    // ==========================================================================
    //                         INTAKE LOGIC (Limelight - Front)
    // ==========================================================================
    private void runIntakeLogic() {
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
    private void runScoreLogic() {
        switch (scoringState) {
            case 0:
                flywheelL.setPower(SCORING_POWER);
                flywheelR.setPower(SCORING_POWER);
                targetSlotIndex = getSlotWithColor(motif.get(motifIndex));
                revolverServo.setPosition(SCORE_POSITIONS[targetSlotIndex]);
                slots.get(targetSlotIndex).isClawOpen = true;
                updateRevolverServos();
                if (actionTimer.getElapsedTimeSeconds() > 0.8) {
                    scoringState = 0;
                    actionTimer.resetTimer();
                }
                break;
            case 1:
                kicker.setPosition(KICKER_FIRE);
                if (actionTimer.getElapsedTimeSeconds() > 0.3) {
                    scoringState = 1;
                    actionTimer.resetTimer();
                }
            case 2:
                kicker.setPosition(KICKER_REST);
                if (actionTimer.getElapsedTimeSeconds() > 1.2) {
                    scoringState = 0;
                    if (allSlotsEmpty()) {
                        scoringState = -1;
                        flywheelL.setPower(0);
                        flywheelR.setPower(0);
                    }
                    actionTimer.resetTimer();

                    slots.get(targetSlotIndex).occupied = false;
                    slots.get(targetSlotIndex).color = "None";
                    motifIndex++;
                    motifIndex = motifIndex % motif.size();
                }
        }
    }

    // ==========================================================================
    // ==========================================================================
    //                             HELPER CLASSES
    // ==========================================================================

    private void initLogic() {
        slots.add(new IntakeSlot(1, claw1));
        slots.add(new IntakeSlot(2, claw2));
        slots.add(new IntakeSlot(3, claw3));

        slots.get(0).occupied = true;
        slots.get(0).color = "Green";
        slots.get(1).occupied = true;
        slots.get(1).color = "Purple";
        slots.get(2).occupied = true;
        slots.get(2).color = "Purple";
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
    private boolean allSlotsEmpty() {
        for (int i = 0; i < 3; i++)
            if (slots.get(i).occupied)
                return false;
        return true;
    }
    private void updateRevolverServos() {
        for (IntakeSlot slot: slots) slot.updateServo();
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
    /**
     * These change the states of the paths and actions. It will also reset the timers of the individual switches
     **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }
    public void runOpMode() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        waitForStart();
        initHardware();
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(new Pose());
        opmodeTimer.resetTimer();
        setPathState(0);

        while (opModeIsActive()) {
            // These loop the movements of the robot, these must be called continuously in order to work
            follower.update();
            switch (pathState) {
                case 0:
                    follower.followPath(toLaunchZone);
                    if (!follower.isBusy()) {
                        setPathState(1);
                        scoringState = 0;
                        actionTimer.resetTimer();
                    }
                    break;
                case 1:
                    runScoreLogic();
                    if (scoringState == -1) {
                        setPathState(2);
                    }
                    break;
                case 2:
                    follower.followPath(toLaunchZone);
                case 3:
                    runIntakeLogic();
                    if (false) {
                        //currentPose =
                        setPathState(4);
                        toLaunchZone = new Path(new BezierLine(startPose, scorePose));
                        toLaunchZone.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());
                    }
                    break;
            }
            follower.setPose(getRobotPoseFromCamera());
            // Feedback to Driver Hub for debugging
            telemetry.addData("path state", pathState);
            telemetry.addData("x", follower.getPose().getX());
            telemetry.addData("y", follower.getPose().getY());
            telemetry.addData("heading", follower.getPose().getHeading());
            telemetry.update();
        }
    }
    class IntakeSlot {
        private final double CLAW_OPEN = 0.0;
        private final double CLAW_CLOSE = 0.17;
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
}
