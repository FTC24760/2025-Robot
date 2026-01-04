package org.firstinspires.ftc.teamcode.teleop;

import static org.firstinspires.ftc.teamcode.SwerveRotationalPID.R_D;
import static org.firstinspires.ftc.teamcode.SwerveRotationalPID.R_I;
import static org.firstinspires.ftc.teamcode.SwerveRotationalPID.R_P;
import static java.lang.Math.abs;
import static java.lang.Math.atan2;
import static java.lang.Math.cos;
import static java.lang.Math.hypot;
import static java.lang.Math.sin;
import static java.lang.Math.toDegrees;
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

// Thank you gemini.
@TeleOp(name="Prototype Teleop (Jan 3, 2026)")
public class PrototypeTeleop extends OpMode {

    // Hardware
    private DcMotorEx leftDrive1, leftDrive2, rightDrive1, rightDrive2;
    private DcMotorEx intakeMotor, middleMotor, leftFlywheel, rightFlywheel;
    private Servo hoodServo, blockerServo;
    private DigitalChannel leftLimitSwitch, rightLimitSwitch;
    private IMU imu;
    private Limelight3A limelight;

    // SWERVES
    private Swerve leftSwerve, rightSwerve;

    // --- Constants ---
    // Adjust these offsets! If the wheel is 45 deg when the magnet hits, put toRadians(45)
    private static final double LEFT_ZERO_OFFSET = toRadians(0);
    private static final double RIGHT_ZERO_OFFSET = toRadians(0);

    private static final double SHOOTER_VELOCITY = 1500; // Ticks per second
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
    private boolean isShootingBatch = false; // true = 3 balls, false = 1 ball

    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime shooterTimer = new ElapsedTime();

    @Override
    public void init() {
        // 1. Drivetrain Motors
        leftDrive1  = hardwareMap.get(DcMotorEx.class, "l1");
        leftDrive2  = hardwareMap.get(DcMotorEx.class, "l2");
        rightDrive1 = hardwareMap.get(DcMotorEx.class, "r1");
        rightDrive2 = hardwareMap.get(DcMotorEx.class, "r2");

        // Directions - preserved from nonfieldorienteddrive
        leftDrive1.setDirection(DcMotorSimple.Direction.REVERSE);
        leftDrive2.setDirection(DcMotorSimple.Direction.REVERSE);
        rightDrive1.setDirection(DcMotorSimple.Direction.REVERSE);
        rightDrive2.setDirection(DcMotorSimple.Direction.REVERSE);

        configureDriveMotor(leftDrive1); configureDriveMotor(leftDrive2);
        configureDriveMotor(rightDrive1); configureDriveMotor(rightDrive2);

        // 2. Mechanisms
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");
        middleMotor = hardwareMap.get(DcMotorEx.class, "middle");
        leftFlywheel = hardwareMap.get(DcMotorEx.class, "leftShooter");
        rightFlywheel = hardwareMap.get(DcMotorEx.class, "rightShooter");

        // Encoder setup for Flywheels
        leftFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFlywheel.setDirection(DcMotorSimple.Direction.REVERSE);

        hoodServo = hardwareMap.get(Servo.class, "hood");
        blockerServo = hardwareMap.get(Servo.class, "blocker");
        blockerServo.setPosition(BLOCKER_CLOSED);

        // 3. Sensors
        leftLimitSwitch = hardwareMap.get(DigitalChannel.class, "leftLimit");
        rightLimitSwitch = hardwareMap.get(DigitalChannel.class, "rightLimit");
        leftLimitSwitch.setMode(DigitalChannel.Mode.INPUT);
        rightLimitSwitch.setMode(DigitalChannel.Mode.INPUT);

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        // 4. Initialize Swerve Objects
        // We pass 0 offset initially, it will be updated in init_loop
        leftSwerve = new Swerve(leftDrive1, leftDrive2, LEFT_ZERO_OFFSET);
        rightSwerve = new Swerve(rightDrive2, rightDrive1, RIGHT_ZERO_OFFSET);

        telemetry.addData("Status", "Initialized. WAITING FOR ZEROING IN INIT_LOOP.");
    }

    private void configureDriveMotor(DcMotorEx motor) {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    // Zeroing routine
    @Override
    public void init_loop() {
        boolean leftDone = zeroModule(leftSwerve, leftLimitSwitch);
        boolean rightDone = zeroModule(rightSwerve, rightLimitSwitch);

        if (leftDone && rightDone) {
            telemetry.addData("Status", "Ready - All Modules Zeroed");
            telemetry.addData("Instructions", "Press Start to Drive");
        } else {
            telemetry.addData("Status", "Zeroing in progress...");
        }

        telemetry.addData("L Limit", leftLimitSwitch.getState());
        telemetry.addData("R Limit", rightLimitSwitch.getState());
    }

    // Logic: Spin module slowly until switch hits, then reset encoder
    private boolean zeroModule(Swerve swerve, DigitalChannel limit) {
        if (swerve.isZeroed) return true;

        // !getState() means magnet is detected (digital touch sensor)
        if (!limit.getState()) {
            swerve.drive1.setPower(0);
            swerve.drive2.setPower(0);

            // Hard Reset of Encoders
            swerve.drive1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            swerve.drive2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            swerve.drive1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            swerve.drive2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            swerve.isZeroed = true;
            return true;
        } else {
            // Rotate the pod. For differential, equal power to both rotates the housing.
            // TODO: Adjust sign if it rotates the wrong way!
            swerve.drive1.setPower(0.2);
            swerve.drive2.setPower(0.2);
            return false;
        }
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
        telemetry.addData("Left Swerve", toDegrees(leftSwerve.wheelAngle));
        telemetry.addData("Right Swerve", toDegrees(rightSwerve.wheelAngle));
        telemetry.update();
    }

    // --- LOGIC METHODS ---

    private void runDriverControl() {
        double driveY = -gamepad1.left_stick_y;
        double driveX = gamepad1.left_stick_x;
        double driveTurn = gamepad1.right_stick_x;

        // Field Centric Math
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double rotX = driveX * cos(-botHeading) - driveY * sin(-botHeading);
        double rotY = driveX * sin(-botHeading) + driveY * cos(-botHeading);

        leftSwerve.update(rotX, rotY, driveTurn);
        rightSwerve.update(rotX, rotY, driveTurn);

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
        middleMotor.setPower(0.5); // Load into middle

        // Field Centric Drive but with Auto-Turn
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        // We assume 'rotX' is 0 because we only want to drive forward towards ball
        double rotY = driveY * cos(-botHeading);
        double rotX = driveY * sin(-botHeading);

        leftSwerve.update(rotX, rotY, turnCmd);
        rightSwerve.update(rotX, rotY, turnCmd);

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
                        double ty = tag.getTargetYDegrees(); // Distance

                        // Turn PID
                        turnCmd = tx * 0.04;
                        if (abs(tx) < 2.0) aligned = true;

                        // Hood Logic: More Up (higher servo val) = Less High?
                        // Logic: "hood more upwards result in less high shoot"
                        // Assuming Servo 1.0 is MAX UP, 0.0 is MAX DOWN.
                        // Far away (ty is near 0) -> Open hood (Low Angle)
                        // Close (ty is negative) -> Close hood (High Angle)
                        // This math depends on your servo linkage!
                        // TODO fix this
                        double hoodPos = 0.5 + (ty * 0.02);
                        hoodServo.setPosition(Math.max(0, Math.min(1, hoodPos)));
                    }
                }

                // Spin Flywheels while aligning
                leftFlywheel.setVelocity(SHOOTER_VELOCITY);
                rightFlywheel.setVelocity(SHOOTER_VELOCITY);

                // Apply Drive
                double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
                leftSwerve.update(0, 0, turnCmd);
                rightSwerve.update(0, 0, turnCmd);

                if (aligned && abs(leftFlywheel.getVelocity() - SHOOTER_VELOCITY) < 200) {
                    shooterState = ShooterState.FIRING;
                    blockerServo.setPosition(BLOCKER_OPEN);
                    shooterTimer.reset();
                }
                break;

            case FIRING:
                // Keep Swerve locked
                leftSwerve.update(0,0,0);
                rightSwerve.update(0,0,0);

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

    // Swerve class (thanks Tony)
    class Swerve {
        private double MOTOR_REDUCTION = 28 * 2.89  * 3.61;
        private double SWERVE_REDUCTION = 6.25;
        private double SPEED_COEFFICIENT = 1;
        DcMotor drive1, drive2;
        double offset;
        public boolean isZeroed = false;

        double targetAngle, wheelAngle, integralRotE, derivativeRotE, lastE = 0;
        double lastTime;

        public Swerve(DcMotor drive1, DcMotor drive2, double offset) {
            this.drive1 = drive1;
            this.drive2 = drive2;
            this.offset = offset;
            lastTime = runtime.milliseconds();
        }

        void update(double rotatedX, double rotatedY, double driveTurn) {
            if (!isZeroed) return; // Safety

            // ---------------- KINEMATICS ----------------
            double deltaX = rotatedX + driveTurn * cos(offset + toRadians(90)); // direction of rotation is 90 degrees from direction to center
            double deltaY = rotatedY + driveTurn * sin(offset + toRadians(90));

            double targetSpeed = hypot(deltaY, deltaX) * SPEED_COEFFICIENT;

            // Calculate Angle (adjusted by zero offset)
            double rawAngle = (drive1.getCurrentPosition() + drive2.getCurrentPosition()) * 0.5
                    / MOTOR_REDUCTION / SWERVE_REDUCTION * toRadians(360);
            wheelAngle = a360(rawAngle - offset);

            if (targetSpeed > 0.01) {
                targetAngle = a360(atan2(deltaY, deltaX) - toRadians(90));
            } else {
                targetAngle = wheelAngle;
            }

            double rotE = a180(targetAngle - wheelAngle);
            targetSpeed *= cos(rotE);

            if (abs(rotE) > toRadians(90))
                rotE = a180(rotE + toRadians(180));

            // ---------------- PID ----------------
            integralRotE += rotE;
            derivativeRotE = (rotE - lastE)/(runtime.milliseconds() - lastTime);

            double rotationPower = (R_P * rotE) + (R_I * integralRotE) + (R_D *derivativeRotE);
            double translationPower = targetSpeed;

            // Motor Outputs
            drive1.setPower(rotationPower + translationPower);
            drive2.setPower(rotationPower - translationPower);

            lastE = rotE;
            lastTime = runtime.milliseconds();
        }

        double a360(double angle) {
            while (angle > toRadians(360)) angle -= toRadians(360);
            while (angle < 0) angle += toRadians(360);
            return angle;
        }
        double a180(double angle) {
            while (angle > toRadians(180)) angle -= toRadians(360);
            while (angle < -180) angle += toRadians(360);
            return angle;
        }
    }
}