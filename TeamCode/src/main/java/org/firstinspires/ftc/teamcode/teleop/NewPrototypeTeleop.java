package org.firstinspires.ftc.teamcode.teleop;

import static java.lang.Math.abs;
import static java.lang.Math.cos;
import static java.lang.Math.hypot;
import static java.lang.Math.signum;
import static java.lang.Math.sin;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.List;

@TeleOp(name="Mecanum Teleop (Final Tuned)")
public class NewPrototypeTeleop extends OpMode {
    boolean isIntaking;

    // --- Hardware ---
    public DcMotorEx flDrive, frDrive, rlDrive, rrDrive;
    public DcMotorEx intakeMotor, middleMotor, leftFlywheel, rightFlywheel;

    // Added blockerServo2 and Lift Servos
    public Servo hoodServo, blocker, blocker2;
    public CRServo liftLeft, liftRight;

    public IMU imu;
    public Limelight3A limelight;

    // --- Constants ---
    // Restored to High Velocity as requested (requires tuned PIDF on motor)
    public static final double CLOSE_SHOOTER_VELOCITY = 2000;
    public static final double SHOOTER_VELOCITY = 1950;
    // Shooter middle motor power - while shooting
    public static final double MIDDLE_SHOOTING_POWER = 0.8;
    // Shooter intake motor power - while shooting
    public static final double INTAKE_SHOOTING_POWER = 0.7;

    // Blocker 1 Constants
    public static final double BLOCKER_OPEN = 0.0;
    public static final double BLOCKER_CLOSED = 0.275;

    // Blocker 2 Constants (Adjust these if the servo is mounted reversed)
    public static final double BLOCKER_2_OPEN = 1.0;
    public static final double BLOCKER_2_CLOSED = 0.858;

    // Alignment Gain from your old code
    public static final double TURN_GAIN = 0.02;
    public static final double MAX_AUTO_TURN = 0.5;

    // Hood Tuning Constants
    // Equation: Hood Pos = HOOD_BASE + (Target_Y_Degrees * HOOD_GAIN)
    public static final double HOOD_BASE = 0.3; // Starting position
    public static final double HOOD_GAIN = 0.0035; // How much to move per degree of distance
    public static final double HOOD_MIN = 0.3;   // Safety Clamp
    public static final double HOOD_MAX = 1.0;   // Safety Clamp

    // Lift Power
    public static final double LIFT_POWER = 1.0;

    // Pipeline IDs
    public static final int PIPELINE_NEURAL = 0; // Game Pieces
    public static final int PIPELINE_TAGS = 1;   // AprilTags
    public static final int PIPELINE_MEGATAG = 3;

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

        // Blockers
        blocker = hardwareMap.get(Servo.class, "blockerL");
        blocker2 = hardwareMap.get(Servo.class, "blockerR");

        blocker.setPosition(BLOCKER_CLOSED);
        blocker2.setPosition(BLOCKER_2_CLOSED);

        // Lift CR Servos
        liftLeft = hardwareMap.get(CRServo.class, "liftLeft");
        liftRight = hardwareMap.get(CRServo.class, "liftRight");
        // Reverse one if they are mounted symmetrically
        liftRight.setDirection(DcMotorSimple.Direction.REVERSE);

        // 3. Sensors
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.DOWN));
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
        isShootingMode = gamepad1.right_bumper || gamepad2.right_bumper;
        isIntaking = (gamepad1.right_trigger > 0.1) || (gamepad2.right_trigger > 0.1);

        leftFlywheel.setVelocity(0);
        rightFlywheel.setVelocity(0);
        intakeMotor.setPower(0);
        middleMotor.setPower(0);
        blocker.setPosition(BLOCKER_CLOSED);
        blocker2.setPosition(BLOCKER_2_CLOSED);

        if (isShootingMode) {
            leftFlywheel.setVelocity(SHOOTER_VELOCITY);
            rightFlywheel.setVelocity(SHOOTER_VELOCITY);
            limelight.pipelineSwitch(PIPELINE_MEGATAG);
            if (gamepad1.left_bumper || gamepad2.left_bumper) { // Fire
                middleMotor.setPower(MIDDLE_SHOOTING_POWER);
                intakeMotor.setPower(INTAKE_SHOOTING_POWER);
                blocker.setPosition(BLOCKER_OPEN);
                blocker2.setPosition(BLOCKER_2_OPEN);
            }
        }

        if (isIntaking) {
            intakeMotor.setPower(0.8);
            middleMotor.setPower(0.8); // Kept existing value, ensuring it runs
        }

        // --- 3. Lift Logic (Operator Control) ---
        if (gamepad2.dpad_up) {
            liftLeft.setPower(LIFT_POWER);
            liftRight.setPower(LIFT_POWER);
        } else if (gamepad2.dpad_down) {
            liftLeft.setPower(-LIFT_POWER);
            liftRight.setPower(-LIFT_POWER);
        } else {
            liftLeft.setPower(0);
            liftRight.setPower(0);
        }

        // --- 4. Hybrid Drive & Auto Align/Hood Logic ---
        double driveY = -gamepad1.left_stick_y;
        double driveX = gamepad1.left_stick_x * 1.1;
        double driveTurn = gamepad1.right_stick_x;

        LLResult result = limelight.getLatestResult();
        boolean targetFound = false;
        double tx = 0;
        double ty = 0; // Vertical offset for Hood

        if (result != null && result.isValid()) {
            if (isShootingMode) {
                // ALIGN TO APRILTAG
                List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
                if (!tags.isEmpty()) {
                    tx = tags.get(0).getTargetXDegrees();
                    ty = tags.get(0).getTargetYDegrees(); // Get distance info
                    targetFound = true;

                    // --- Auto Hood Adjustment ---
                    // Calculates position based on Y-angle (Distance)
                    double calculatedHoodPos = HOOD_BASE + (ty * HOOD_GAIN);

                    // Clamp to safety limits
                    calculatedHoodPos = Math.max(HOOD_MIN, Math.min(HOOD_MAX, calculatedHoodPos));
                    hoodServo.setPosition(calculatedHoodPos);
                }
            } else if (isIntaking) {
                // ALIGN TO GAME PIECE
                List<LLResultTypes.DetectorResult> detections = result.getDetectorResults();
                if (!detections.isEmpty()) {
                    tx = detections.get(0).getTargetXDegrees();
                    targetFound = true;
                }
            }
        }

        // Apply Turn Override if Target Found
        if (targetFound) {
            double autoTurn = tx * TURN_GAIN;
            driveTurn = Math.max(-MAX_AUTO_TURN, Math.min(MAX_AUTO_TURN, autoTurn));
        }

        setMecanumPower(driveX, driveY, driveTurn);

        // --- 5. Telemetry ---
        telemetry.addData("Mode", isShootingMode ? "SHOOTING" : (isIntaking ? "INTAKING" : "DRIVER"));
        telemetry.addData("Target Found", targetFound);
        telemetry.addData("Limelight TX", tx);
        telemetry.addData("Limelight TY", ty);
        telemetry.addData("Hood Pos", hoodServo.getPosition());
        telemetry.addData("Shooter Vel", leftFlywheel.getVelocity());
        telemetry.update();
    }

    public void setMecanumPower(double strafe, double forward, double turn) {
        double F_RATIO = 0.12;
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Field Centric Rotation
        double rotX = strafe * cos(-botHeading) - forward * sin(-botHeading);
        double rotY = strafe * sin(-botHeading) + forward * cos(-botHeading);
        double hypotenuse = hypot(rotX, rotY);
        if (hypotenuse > 0) {
            rotX = (1 - F_RATIO) * rotX + F_RATIO * rotX / hypotenuse;
            rotX = (1 - F_RATIO) * rotX + F_RATIO * rotX / hypotenuse;
        }
        else if (turn != 0) {
            turn = (1 - F_RATIO) * turn + F_RATIO * signum(turn);
        }
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