package org.firstinspires.ftc.teamcode.teleop;

import static java.lang.Math.abs;
import static java.lang.Math.cos;
import static java.lang.Math.sin;
import static java.lang.Math.toRadians;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.List;

@TeleOp(name="Mecanum Teleop (Jan 27, 2026)")
public class PrototypeTeleop extends OpMode {

    // --- Hardware ---
    // Drive Motors (SWYFT / Mecanum)
    private DcMotorEx flDrive, frDrive, rlDrive, rrDrive;

    // Mechanisms (Unchanged)
    private DcMotorEx intakeMotor, middleMotor, leftFlywheel, rightFlywheel;
    private Servo hoodServo, blockerServo;
    private DigitalChannel leftLimitSwitch, rightLimitSwitch; // Kept references, though not needed for zeroing anymore
    private IMU imu;
    private Limelight3A limelight;

    // --- Constants ---
    private static final double SHOOTER_VELOCITY = 2000; // Ticks per second (Adjusted to a realistic encoder value)
    private static final double BLOCKER_OPEN = 0.5;
    private static final double BLOCKER_CLOSED = 0.0;

    // State machine - robot states
    private enum RobotMode {
        DRIVER_CONTROL,
        AUTO_INTAKE,
        AUTO_SHOOT
    }

    private enum ShooterState {
        IDLE,
        ALIGNING,
        SPINNING_UP,
        FIRING,
        RESET
    }

    private RobotMode currentMode = RobotMode.DRIVER_CONTROL;
    private ShooterState shooterState = ShooterState.IDLE;
    private boolean isShootingBatch = false;

    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime shooterTimer = new ElapsedTime();

    @Override
    public void init() {
        // 1. Drivetrain Motors (Mecanum Setup)
        // Naming matches your reference: flDrive, rlDrive, frDrive, rrDrive
        flDrive = hardwareMap.get(DcMotorEx.class, "flDrive");
        rlDrive = hardwareMap.get(DcMotorEx.class, "rlDrive");
        frDrive = hardwareMap.get(DcMotorEx.class, "frDrive");
        rrDrive = hardwareMap.get(DcMotorEx.class, "rrDrive");

        // Directions: usually Left is Reverse, Right is Forward for standard Mecanum
        flDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rlDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        frDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        rrDrive.setDirection(DcMotorSimple.Direction.FORWARD);

        // Anti-Drift: Use Encoders to maintain velocity
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

        leftFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFlywheel.setDirection(DcMotorSimple.Direction.REVERSE);

        hoodServo = hardwareMap.get(Servo.class, "hood");
        blockerServo = hardwareMap.get(Servo.class, "blocker");
        blockerServo.setPosition(BLOCKER_CLOSED);

        // 3. Sensors
        // Swerve zeroing switches are technically not needed for Mecanum, but keeping init safe
        try {
            leftLimitSwitch = hardwareMap.get(DigitalChannel.class, "leftLimit");
            rightLimitSwitch = hardwareMap.get(DigitalChannel.class, "rightLimit");
        } catch (Exception e) {
            // Switches might not exist on Mecanum bot
        }

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        telemetry.addData("Status", "Initialized - Ready for Start");
    }

    private void configureDriveMotor(DcMotorEx motor) {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // CRITICAL FOR DRIFT CORRECTION:
        // RUN_USING_ENCODER uses the internal PID to ensure the wheel actually spins
        // at the requested speed, compensating for friction differences.
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void init_loop() {
        // No swerve zeroing needed!
        telemetry.addData("Status", "Ready");
    }

    @Override
    public void start() {
        runtime.reset();
        limelight.pipelineSwitch(0);
    }

    @Override
    public void loop() {
        // 1. Global Inputs
        if (gamepad1.options) imu.resetYaw();

        // Switch Modes
        if (gamepad1.right_trigger > 0.1) {
            if (currentMode != RobotMode.AUTO_INTAKE) {
                currentMode = RobotMode.AUTO_INTAKE;
                limelight.pipelineSwitch(0); // Neural artifact detection pipeline
            }
        } else if (gamepad1.left_bumper) { // Shoot ONE
            startShootingSequence(false);
        } else if (gamepad1.right_bumper) { // Shoot ALL
            startShootingSequence(true);
        } else if (gamepad1.b) { // Cancel
            currentMode = RobotMode.DRIVER_CONTROL;
            shooterState = ShooterState.IDLE;
            stopMechanisms();
        }

        // --- 2. Main Logic ---
        switch (currentMode) {
            case DRIVER_CONTROL:
                runDriverControl();
                break;
            case AUTO_INTAKE:
                runAutoIntake();
                break;
            case AUTO_SHOOT:
                runAutoShoot();
                break;
        }

        // --- 3. Telemetry ---
        telemetry.addData("Mode", currentMode);
        telemetry.addData("Shooter State", shooterState);
        telemetry.update();
    }

    // --- LOGIC METHODS ---

    /**
     * Universal drive method for Field Oriented Mecanum
     * @param strafe speed (+ is right)
     * @param forward speed (+ is forward)
     * @param turn speed (+ is right/CW)
     */
    private void setMecanumPower(double strafe, double forward, double turn) {
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Field Centric Rotation
        double rotX = strafe * cos(-botHeading) - forward * sin(-botHeading);
        double rotY = strafe * sin(-botHeading) + forward * cos(-botHeading);

        // Mecanum Kinematics
        // denominator ensures we don't exceed power limit (1.0) while maintaining ratios
        double denominator = Math.max(abs(rotY) + abs(rotX) + abs(turn), 1);

        double flPower = (rotY + rotX + turn) / denominator;
        double rlPower = (rotY - rotX + turn) / denominator;
        double frPower = (rotY - rotX - turn) / denominator;
        double rrPower = (rotY + rotX - turn) / denominator;

        flDrive.setPower(flPower);
        rlDrive.setPower(rlPower);
        frDrive.setPower(frPower);
        rrDrive.setPower(rrPower);
    }

    private void runDriverControl() {
        double driveY = -gamepad1.left_stick_y; // Remember Y stick is reversed
        double driveX = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing friction
        double driveTurn = gamepad1.right_stick_x;

        setMecanumPower(driveX, driveY, driveTurn);

        // Manual Intake Override
        if (gamepad1.x) intakeMotor.setPower(0.8);
        else intakeMotor.setPower(0);
    }

    private void runAutoIntake() {
        // Driver controls Forward/Back speed manually to approach
        // Robot controls Turning to align with ball
        double driveY = -gamepad1.left_stick_y;

        LLResult result = limelight.getLatestResult();
        double turnCmd = 0;

        if (result != null && result.isValid()) {
            LLResultTypes.DetectorResult detection = result.getDetectorResults().stream().findFirst().orElse(null);
            if (detection != null) {
                turnCmd = detection.getTargetXDegrees() * 0.03; // Simple P align
            }
        }

        // Intake on
        intakeMotor.setPower(1.0);
        middleMotor.setPower(0.5);

        // Auto Drive: Manual Forward/Back, Zero Strafe, Auto Turn
        setMecanumPower(0, driveY, turnCmd);

        // Exit condition
        if (gamepad1.right_trigger < 0.1) {
            currentMode = RobotMode.DRIVER_CONTROL;
            stopMechanisms();
        }
    }

    private void startShootingSequence(boolean batch) {
        currentMode = RobotMode.AUTO_SHOOT;
        shooterState = ShooterState.ALIGNING;
        isShootingBatch = batch;
        limelight.pipelineSwitch(1); // AprilTag Pipeline
    }

    private void runAutoShoot() {
        switch (shooterState) {
            case ALIGNING:
                // 1. Align Robot & Hood
                LLResult result = limelight.getLatestResult();
                double turnCmd = 0;
                boolean aligned = false;

                if (result != null && result.isValid()) {
                    List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
                    if (!tags.isEmpty()) {
                        LLResultTypes.FiducialResult tag = tags.get(0);
                        double tx = tag.getTargetXDegrees();
                        double ty = tag.getTargetYDegrees();

                        // Turn PID
                        turnCmd = tx * 0.04;
                        if (abs(tx) < 2.0) aligned = true;

                        // Hood Logic
                        double hoodPos = 0.5 + (ty * 0.02);
                        hoodServo.setPosition(Math.max(0, Math.min(1, hoodPos)));
                    }
                }

                // Spin Flywheels while aligning
                leftFlywheel.setVelocity(SHOOTER_VELOCITY);
                rightFlywheel.setVelocity(SHOOTER_VELOCITY);

                // Apply Drive (Stop strafe/fwd, just turn)
                setMecanumPower(0, 0, turnCmd);

                if (aligned && abs(leftFlywheel.getVelocity() - SHOOTER_VELOCITY) < 200) {
                    shooterState = ShooterState.FIRING;
                    blockerServo.setPosition(BLOCKER_OPEN);
                    shooterTimer.reset();
                }
                break;

            case FIRING:
                // Lock Drivetrain
                setMecanumPower(0, 0, 0);

                // Keep Spinning
                leftFlywheel.setVelocity(SHOOTER_VELOCITY);
                rightFlywheel.setVelocity(SHOOTER_VELOCITY);

                // Feed
                middleMotor.setPower(0.5);

                // Exit conditions
                if (isShootingBatch) {
                    if (shooterTimer.milliseconds() > 2000) shooterState = ShooterState.RESET;
                } else {
                    if (shooterTimer.milliseconds() > 500) shooterState = ShooterState.RESET;
                }
                break;

            case RESET:
                stopMechanisms();
                currentMode = RobotMode.DRIVER_CONTROL;
                shooterState = ShooterState.IDLE;
                break;
        }
    }

    private void stopMechanisms() {
        intakeMotor.setPower(0);
        middleMotor.setPower(0);
        leftFlywheel.setPower(0);
        rightFlywheel.setPower(0);
        blockerServo.setPosition(BLOCKER_CLOSED);
    }
}