package org.firstinspires.ftc.teamcode.teleop;

import static java.lang.Math.atan2;
import static java.lang.Math.hypot;
import static java.lang.Math.toRadians;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


@TeleOp(name="Pedro TeleOp")
public class PedroTeleopRed extends NewPrototypeTeleop {

    Pose3D oldPose = new Pose3D(new Position(DistanceUnit.INCH, 0, 0, 0, 0), new YawPitchRollAngles(AngleUnit.RADIANS, 0, 0, 0 ,0));
    public static final Pose frontScorePose = new Pose(84, 84, toRadians(45));
    public static final Pose backScorePose = new Pose(84, 12, toRadians(68.2));
    public static final Pose frontAutoParkPose = new Pose(96, 60, toRadians(90));
    public static final Pose backAutoParkPose = new Pose(108, 12, toRadians(90));
    public String scoringFrontBack = "Front";
    public Follower follower;
    public Pose scorePose = frontScorePose;
    double flyWheelSpeed, intakeMotorSpeed, middleMotorSpeed;
    boolean isBlockerOpen;
    @Override
    public void init() {
        super.init(); // from newprototypeteleop
        follower = Constants.createFollower(hardwareMap);
    }

    @Override
    public void loop() {
        updatePose(follower);

        if (gamepad1.dpad_up) scoringFrontBack = "Front";
        if (gamepad1.dpad_down) scoringFrontBack = "Back";
        if (gamepad2.dpad_up) follower.setPose(frontAutoParkPose);
        if (gamepad2.dpad_down) follower.setPose(backAutoParkPose);

        // --- 1. Global Controls ---
        if (gamepad1.options) imu.resetYaw();

        // --- 2. Mode Selection ---
        isShootingMode = gamepad1.right_bumper || gamepad2.right_bumper;
        isIntaking = (gamepad1.right_trigger > 0.1) || (gamepad2.right_trigger > 0.1);

        // Mechanism Power Control
        resetMotors();
        if (isShootingMode) shootingLogic(gamepad1.left_bumper || gamepad2.left_bumper);
        if (isIntaking) intakeLogic();
        updateMotors();

        // --- 3. Lift Logic (Operator Control) ---
        liftLeft.setPower(0);
        liftRight.setPower(0);
        if (gamepad2.dpad_up) {
            liftLeft.setPower(LIFT_POWER);
            liftRight.setPower(LIFT_POWER);
        }
        if (gamepad2.dpad_down) {
            liftLeft.setPower(-LIFT_POWER);
            liftRight.setPower(-LIFT_POWER);
        }

        // --- 4. Hybrid Drive & Auto Align/Hood Logic ---
        double driveY = -gamepad1.left_stick_y;
        double driveX = gamepad1.left_stick_x * 1.1;
        double driveTurn;
        double targetAngle = 0; double botHeading = 0;
        if (hypot(gamepad1.right_stick_x, gamepad1.right_stick_y) > 0.1) {
            targetAngle = a360(atan2(-gamepad1.right_stick_y, gamepad1.right_stick_x));
            botHeading = a360(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
            driveTurn = a180(targetAngle - botHeading) * 0.05;
        }
        else driveTurn = 0;

        if (isShootingMode) {
            if (scoringFrontBack.equals("Front")) scorePose = frontScorePose;
            else scorePose = backScorePose;
            follower.holdPoint(scorePose);
        } else setMecanumPower(driveX, driveY, driveTurn);

        telemetry.addData("PedroPathing score position", scoringFrontBack);
        // --- 5. Telemetry ---
        telemetry.addData("Mode", isShootingMode ? "SHOOTING" : (isIntaking ? "INTAKING" : "DRIVER"));
        telemetry.addData("Hood Pos", hoodServo.getPosition());
        telemetry.addData("Shooter Vel", leftFlywheel.getVelocity());
        telemetry.addData("Target angle", targetAngle);
        telemetry.addData("Bot Heading", botHeading);
        telemetry.update();
    }
    public void updatePose(Follower follower) {
        //limelight.updateRobotOrientation(imu.getRobotYawPitchRollAngles().getYaw());
        limelight.pipelineSwitch(PIPELINE_MEGATAG);
        LLResult result = limelight.getLatestResult();

        if (result.getBotposeTagCount() > 0) {
            //Pose3D robotPose = result.getBotpose_MT2();
            Pose3D robotPose = result.getBotpose();
            Position robotPosition = robotPose.getPosition();
            YawPitchRollAngles robotOrientation = robotPose.getOrientation();
            if (robotPose != oldPose) {
                follower.setPose(new Pose(robotPosition.y + 72, 72 - robotPosition.x, robotOrientation.getYaw()));
                oldPose = robotPose;
            }
        }
        // if our angular velocity is greater than 360 degrees per second, ignore vision updates
        /*if(Math.abs(m_gyro.getRate()) > 360)
        {
            doRejectUpdate = true;
        }
        if(mt2.tagCount == 0)
        {
            doRejectUpdate = true;
        }
        if(!doRejectUpdate)
        {
            m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
            m_poseEstimator.addVisionMeasurement(
                    mt2.pose,
                    mt2.timestampSeconds);
        }*/
    }
    public void resetMotors() {
        flyWheelSpeed = 0;
        intakeMotorSpeed = 0;
        middleMotorSpeed = 0;
        isBlockerOpen = false;
    }
    public void shootingLogic(boolean fire) {
        flyWheelSpeed = SHOOTER_VELOCITY;
        limelight.pipelineSwitch(PIPELINE_MEGATAG);
        if (fire) {
            middleMotorSpeed = 1.0;
            intakeMotorSpeed = INTAKE_SHOOTING_POWER;
            isBlockerOpen = true;
        }
    }
    public void intakeLogic() {
        intakeMotorSpeed = 1;
        middleMotorSpeed = 0.8;
    }
    public void updateMotors() {
        leftFlywheel.setVelocity(flyWheelSpeed);
        rightFlywheel.setVelocity(flyWheelSpeed);
        intakeMotor.setPower(intakeMotorSpeed);
        middleMotor.setPower(middleMotorSpeed);
        if (isBlockerOpen) {
            blocker.setPosition(BLOCKER_OPEN);
            blocker2.setPosition(BLOCKER_2_OPEN);
        }
        else {
            blocker.setPosition(BLOCKER_CLOSED);
            blocker2.setPosition(BLOCKER_2_CLOSED);
        }

    }
    double a180(double angle) {
        while (angle > toRadians(180)) angle -= toRadians(360);
        while (angle < -180) angle += toRadians(360);
        return angle;
    }
    double a360(double angle) {
        while (angle > toRadians(360)) angle -= toRadians(360);
        while (angle < 0) angle += toRadians(360);
        return angle;
    }
}
