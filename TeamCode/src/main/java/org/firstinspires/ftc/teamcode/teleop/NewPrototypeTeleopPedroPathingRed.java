package org.firstinspires.ftc.teamcode.teleop;

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
public class NewPrototypeTeleopPedroPathingRed extends NewPrototypeTeleop {
    Pose3D oldPose = new Pose3D(new Position(DistanceUnit.INCH, 0, 0, 0, 0), new YawPitchRollAngles(AngleUnit.RADIANS, 0, 0, 0 ,0));
    public static final Pose redFrontScorePose = new Pose(84, 84, toRadians(45));
    public static final Pose redBackScorePose = new Pose(84, 12, toRadians(68.2));
    public String scoringFrontBack = "Front";
    public Follower follower = Constants.createFollower(hardwareMap);
    public Pose scorePose = redFrontScorePose;

    @Override
    public void loop() {
        updatePose(follower);

        if (gamepad1.dpad_up) scoringFrontBack = "Front";
        if (gamepad1.dpad_down) scoringFrontBack = "Back";
        if (gamepad2.dpad_up) follower.setPose(new Pose(96, 60, 90));
        if (gamepad2.dpad_down) follower.setPose(new Pose(108, 12, 90));

        // --- 1. Global Controls ---
        if (gamepad1.options) imu.resetYaw();

        // --- 2. Mode Selection ---
        isShootingMode = gamepad1.right_bumper || gamepad2.right_bumper;
        isIntaking = (gamepad1.right_trigger > 0.1) || (gamepad2.right_trigger > 0.1);

        // Mechanism Power Control
        resetMotors();

        if (isShootingMode) {
            shootingLogic(gamepad1.left_bumper || gamepad2.left_bumper);
        }

        if (isIntaking) {
            intakeLogic();
        }


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
        double driveTurn = gamepad1.right_stick_x;

        if (isShootingMode) {
            if (scoringFrontBack.equals("Front")) scorePose = redFrontScorePose;
            else scorePose = redBackScorePose;
            follower.holdPoint(scorePose);
        } else setMecanumPower(driveX, driveY, driveTurn);

        telemetry.addData("PedroPathing score position", scoringFrontBack);
        // --- 5. Telemetry ---
        telemetry.addData("Mode", isShootingMode ? "SHOOTING" : (isIntaking ? "INTAKING" : "DRIVER"));
        telemetry.addData("Hood Pos", hoodServo.getPosition());
        telemetry.addData("Shooter Vel", leftFlywheel.getVelocity());
        telemetry.update();
    }
    void updatePose(Follower follower) {
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
    void resetMotors() {
        leftFlywheel.setVelocity(0);
        rightFlywheel.setVelocity(0);
        intakeMotor.setPower(0);
        middleMotor.setPower(0);
        blocker.setPosition(BLOCKER_CLOSED);
        blocker2.setPosition(BLOCKER_2_CLOSED);
    }
    void shootingLogic(boolean fire) {
        leftFlywheel.setVelocity(SHOOTER_VELOCITY);
        rightFlywheel.setVelocity(SHOOTER_VELOCITY);
        limelight.pipelineSwitch(PIPELINE_MEGATAG);
        if (fire) {
            middleMotor.setPower(MIDDLE_SHOOTING_POWER);
            intakeMotor.setPower(INTAKE_SHOOTING_POWER);
            blocker.setPosition(BLOCKER_OPEN);
            blocker2.setPosition(BLOCKER_2_OPEN);
        }
    }
    void intakeLogic() {
        intakeMotor.setPower(0.8);
        middleMotor.setPower(0.8); // Kept existing value, ensuring it runs
    }
}
