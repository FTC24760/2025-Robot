package org.firstinspires.ftc.teamcode.teleop;

import static java.lang.Math.abs;
import static java.lang.Math.cos;
import static java.lang.Math.sin;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.List;

@TeleOp(name="Mecanum Teleop (Final Tuned)")
public class PrototypeTeleop extends OpMode {

    // --- Hardware ---
    public DcMotorEx flDrive, frDrive, rlDrive, rrDrive;
    public DcMotorEx intakeMotor, middleMotor, leftFlywheel, rightFlywheel;
    public Servo hoodServo, blockerServo;
    public IMU imu;
    public Limelight3A limelight;

    // --- Constants ---
    // Restored to High Velocity as requested (requires tuned PIDF on motor)
    public static final double SHOOTER_VELOCITY = 999999;

    public static final double BLOCKER_OPEN = 0.75;
    public static final double BLOCKER_CLOSED = 1.0;

    // Alignment Gain from your old code
    public static final double TURN_GAIN = 0.02;
    public static final double MAX_AUTO_TURN = 0.5;

    // Pipeline IDs
    public static final int PIPELINE_NEURAL = 0; // Game Pieces
    public static final int PIPELINE_TAGS = 1;   // AprilTags

    // Hood servo position
    public double hoodPosition = 0.0;

    public boolean isShootingMode = false;
    public void initHardware() {
        flDrive = hardwareMap.get(DcMotorEx.class, "flDrive");
        rlDrive = hardwareMap.get(DcMotorEx.class, "rlDrive");
        frDrive = hardwareMap.get(DcMotorEx.class, "frDrive");
        rrDrive = hardwareMap.get(DcMotorEx.class, "rrDrive");

        flDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rlDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        frDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        rrDrive.setDirection(DcMotorSimple.Direction.FORWARD);

        configureDriveMotor(flDrive);
        configureDriveMotor(rlDrive);
        configureDriveMotor(frDrive);
        configureDriveMotor(rrDrive);

        // 2. Mechanisms
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");
        middleMotor = hardwareMap.get(DcMotorEx.class, "middle");
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        middleMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFlywheel = hardwareMap.get(DcMotorEx.class, "leftShooter");
        rightFlywheel = hardwareMap.get(DcMotorEx.class, "rightShooter");

        // CRITICAL: Set to RUN_USING_ENCODER for setVelocity to work
        leftFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFlywheel.setDirection(DcMotorSimple.Direction.REVERSE);

        hoodServo = hardwareMap.get(Servo.class, "hood");
        blockerServo = hardwareMap.get(Servo.class, "blocker");
        blockerServo.setPosition(BLOCKER_CLOSED);

        // 3. Sensors
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        imu.initialize(parameters);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(PIPELINE_NEURAL);
        limelight.start();
    }
    @Override
    public void init() {
        // 1. Drivetrain
        initHardware();

        telemetry.addData("Status", "Initialized - Velocity Mode Active");
    }

    public void configureDriveMotor(DcMotorEx motor) {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void start() {
        limelight.pipelineSwitch(PIPELINE_NEURAL);
    }

    @Override
    public void loop() {
        // --- 1. Global Controls ---
        if (gamepad1.options) imu.resetYaw();

        // --- 2. Mode Selection ---
        // Right Bumper = Shooting Mode (Tag Align)
        // Right Trigger = Intake Mode (Game Piece Align)
        // Default = Driver Control (No Align)

        isShootingMode = gamepad1.right_bumper || gamepad2.right_bumper;
        boolean isIntaking = (gamepad1.right_trigger > 0.1) || (gamepad2.right_trigger > 0.1);

        // Mechanism Power Control
        if (isShootingMode) {
            leftFlywheel.setVelocity(SHOOTER_VELOCITY);
            rightFlywheel.setVelocity(SHOOTER_VELOCITY);
            limelight.pipelineSwitch(PIPELINE_TAGS);
        } else {
            leftFlywheel.setVelocity(0);
            rightFlywheel.setVelocity(0);
            // If intaking, use Neural pipeline, otherwise Neural (default)
            limelight.pipelineSwitch(PIPELINE_NEURAL);
        }

        if (isIntaking) {
            intakeMotor.setPower(0.8);
            middleMotor.setPower(0.3);
            blockerServo.setPosition(BLOCKER_CLOSED);
        } else if (isShootingMode && (gamepad1.left_bumper || gamepad2.left_bumper)) { // Fire
            middleMotor.setPower(1.0);
            blockerServo.setPosition(BLOCKER_OPEN);
            intakeMotor.setPower(0);
        } else {
            intakeMotor.setPower(0);
            middleMotor.setPower(0);
            blockerServo.setPosition(BLOCKER_CLOSED);
        }

        // --- 3. Hybrid Drive & Auto Align Logic ---

        double driveY = -gamepad1.left_stick_y; // Forward
        double driveX = gamepad1.left_stick_x * 1.1; // Strafe
        double driveTurn = gamepad1.right_stick_x; // Manual Turn (Default)

        LLResult result = limelight.getLatestResult();
        boolean targetFound = false;
        double tx = 0;

        // Determine if we should Auto-Align
        if (result != null && result.isValid()) {
            if (isShootingMode) {
                // ALIGN TO APRILTAG
                List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
                if (!tags.isEmpty()) {
                    tx = tags.get(0).getTargetXDegrees();
                    targetFound = true;
                }
            } else if (isIntaking) {
                // ALIGN TO GAME PIECE (Neural/Color)
                List<LLResultTypes.DetectorResult> detections = result.getDetectorResults();
                if (!detections.isEmpty()) {
                    tx = detections.get(0).getTargetXDegrees();
                    targetFound = true;
                }
            }
        }

        // Apply Turn Override if Target Found
        if (targetFound) {
            // Using your simple P-loop logic: turn = error * gain
            double autoTurn = tx * TURN_GAIN;

            // Optional: Clamp max speed so it doesn't whip around too fast
            driveTurn = Math.max(-MAX_AUTO_TURN, Math.min(MAX_AUTO_TURN, autoTurn));
        }

        setMecanumPower(driveX, driveY, driveTurn);

        // --- 4. Telemetry ---
        telemetry.addData("Mode", isShootingMode ? "SHOOTING (Tag)" : (isIntaking ? "INTAKING (Neural)" : "DRIVER"));
        telemetry.addData("Target Found", targetFound);
        telemetry.addData("TX", tx);
        telemetry.addData("Flywheel Velocity", leftFlywheel.getVelocity());
        telemetry.update();
    }

    public void setMecanumPower(double strafe, double forward, double turn) {
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Field Centric Rotation
        double rotX = strafe * cos(-botHeading) - forward * sin(-botHeading);
        double rotY = strafe * sin(-botHeading) + forward * cos(-botHeading);

        double denominator = Math.max(abs(rotY) + abs(rotX) + abs(turn), 1);
        flDrive.setPower((rotY + rotX + turn) / denominator);
        rlDrive.setPower((rotY - rotX + turn) / denominator);
        frDrive.setPower((rotY - rotX - turn) / denominator);
        rrDrive.setPower((rotY + rotX - turn) / denominator);
    }
    Pose getAprilTagLocalization() {
        return new Pose();
    }
}