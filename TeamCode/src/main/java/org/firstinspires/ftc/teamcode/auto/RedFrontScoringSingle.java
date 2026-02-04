package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Red Single Score", group = "Auto")
public class RedFrontScoringSingle extends AutoExample {
    public static Pose startPose = new Pose(122, 122, Math.toRadians(45));
    public static Pose scorePose = new Pose(84.000, 84.000, Math.toRadians(45));

    public static Pose intake1Pose = new Pose(85, 84, Math.toRadians(0));
    public static Pose intake1GrabPose = new Pose(116, 84, Math.toRadians(0));

    public static Pose intake2Pose = new Pose(85, 60, Math.toRadians(0));
    public static Pose intake2GrabPose = new Pose(118, 60, Math.toRadians(0));

    public static Pose intake3Pose = new Pose(85, 36, Math.toRadians(0));
    public static Pose intake3GrabPose = new Pose(120, 36, Math.toRadians(0));

    public static Pose parkPose = new Pose(96, 60, Math.toRadians(90));

    public static double THRESHOLD = 1.0; // threshold for location thingy

    public DcMotorEx intakeMotor, middleMotor, leftFlywheel, rightFlywheel;
    public Servo hoodServo, blockerServo;

    public static class Paths {
        public static Path Path1, PathToIntake1, PathGrab1, PathScore1;
        public static Path PathToIntake2, PathGrab2, PathScore2;
        public static Path PathToIntake3, PathGrab3, PathScore3;
        public static Path PathPark;

        public Paths(Follower follower) {
            Path1 = new Path(new BezierLine(startPose, scorePose));
            Path1.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

            PathToIntake1 = new Path(new BezierLine(scorePose, intake1Pose));
            PathToIntake1.setLinearHeadingInterpolation(scorePose.getHeading(), intake1Pose.getHeading());

            PathGrab1 = new Path(new BezierLine(intake1Pose, intake1GrabPose));
            PathGrab1.setLinearHeadingInterpolation(intake1Pose.getHeading(), intake1GrabPose.getHeading());

            PathScore1 = new Path(new BezierLine(intake1GrabPose, scorePose));
            PathScore1.setLinearHeadingInterpolation(intake1GrabPose.getHeading(), scorePose.getHeading());

            PathToIntake2 = new Path(new BezierLine(scorePose, intake2Pose));
            PathToIntake2.setLinearHeadingInterpolation(scorePose.getHeading(), intake2Pose.getHeading());

            PathGrab2 = new Path(new BezierLine(intake2Pose, intake2GrabPose));
            PathGrab2.setLinearHeadingInterpolation(intake2Pose.getHeading(), intake2GrabPose.getHeading());

            PathScore2 = new Path(new BezierLine(intake2GrabPose, scorePose));
            PathScore2.setLinearHeadingInterpolation(intake2GrabPose.getHeading(), scorePose.getHeading());

            PathToIntake3 = new Path(new BezierLine(scorePose, intake3Pose));
            PathToIntake3.setLinearHeadingInterpolation(scorePose.getHeading(), intake3Pose.getHeading());

            PathGrab3 = new Path(new BezierLine(intake3Pose, intake3GrabPose));
            PathGrab3.setLinearHeadingInterpolation(intake3Pose.getHeading(), intake3GrabPose.getHeading());

            PathScore3 = new Path(new BezierLine(intake3GrabPose, scorePose));
            PathScore3.setLinearHeadingInterpolation(intake3GrabPose.getHeading(), scorePose.getHeading());

            PathPark = new Path(new BezierLine(scorePose, parkPose));
            PathPark.setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading());
        }
    }

    public void initHardware() {
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
        blockerServo.setPosition(1.0);
    }

    public boolean isAtPose(Pose target) {
        double distance = Math.hypot(
                follower.getPose().getX() - target.getX(),
                follower.getPose().getY() - target.getY()
        );
        return distance < THRESHOLD;
    }

    @Override
    public void init() {
        initHardware();
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        actionTimer = new Timer();
        Paths paths = new Paths(follower);
    }
    @Override
    public void start() {

        actionTimer.resetTimer();
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        opmodeTimer.resetTimer();
        pathState = 0;
    }

    @Override
    public void loop() {
        follower.update();
        switch (pathState) {
            case 0:
                follower.followPath(Paths.Path1);
                if (isAtPose(scorePose)) {
                    pathState = 1;
                    actionTimer.resetTimer();
                }
                break;
            case 1:
                follower.holdPoint(scorePose);
                runScoreLogic(true);
                if (actionTimer.getElapsedTimeSeconds() > 3) {
                    runScoreLogic(false);
                    pathState = 2;
                }
                break;
            case 2:
                follower.followPath(Paths.PathToIntake1);
                double headingError = Math.abs(follower.getPose().getHeading() - intake1Pose.getHeading());
                if (headingError < 0.07) {
                    pathState = 3;
                    actionTimer.resetTimer();
                }
                break;
            case 3:
                runIntakeLogic(true);
                follower.followPath(Paths.PathGrab1);
                if (isAtPose(intake1GrabPose)) {
                    pathState = 4;
                    actionTimer.resetTimer();
                }
                break;
            case 4:
                runIntakeLogic(false);
                follower.followPath(Paths.PathScore1);
                if (isAtPose(scorePose)) {
                    pathState = 5;
                    actionTimer.resetTimer();
                }
                break;
            case 5:
                follower.holdPoint(scorePose);
                runScoreLogic(true);
                if (actionTimer.getElapsedTimeSeconds() > 3) {
                    runScoreLogic(false);
                    pathState = 6;
                }
                break;
            case 6:
                follower.followPath(Paths.PathToIntake2);
                if (isAtPose(intake2Pose)) {
                    pathState = 7;
                    actionTimer.resetTimer();
                }
                break;
            case 7:
                runIntakeLogic(true);
                follower.followPath(Paths.PathGrab2);
                if (isAtPose(intake2GrabPose)) {
                    pathState = 8;
                }
                break;
            case 8:
                runIntakeLogic(false);
                follower.followPath(Paths.PathScore2);
                if (isAtPose(scorePose)) {
                    pathState = 9;
                    actionTimer.resetTimer();
                }
                break;
            case 9:
                follower.holdPoint(scorePose);
                runScoreLogic(true);
                if (actionTimer.getElapsedTimeSeconds() > 3) {
                    runScoreLogic(false);
                    pathState = 10;
                }
                break;
            case 10:
                follower.followPath(Paths.PathToIntake3);
                if (isAtPose(intake3Pose)) {
                    pathState = 11;
                    actionTimer.resetTimer();
                }
                break;
            case 11:
                runIntakeLogic(true);
                follower.followPath(Paths.PathGrab3);
                if (isAtPose(intake3GrabPose)) {
                    pathState = 12;
                }
                break;
            case 12:
                runIntakeLogic(false);
                follower.followPath(Paths.PathScore3);
                if (isAtPose(scorePose)) {
                    pathState = 13;
                    actionTimer.resetTimer();
                }
                break;
            case 13:
                follower.holdPoint(scorePose);
                runScoreLogic(true);
                if (actionTimer.getElapsedTimeSeconds() > 3) {
                    runScoreLogic(false);
                    pathState = 14;
                }
                break;
            case 14:
                follower.followPath(Paths.PathPark);
                if (!follower.isBusy() || actionTimer.getElapsedTimeSeconds() > 5) {
                    pathState = 15;
                    actionTimer.resetTimer();
                }
                break;
            case 15:
                follower.holdPoint(parkPose);
                break;
        }

        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    public void runScoreLogic(boolean active) {
        if (active) {
            leftFlywheel.setVelocity(SHOOTER_VELOCITY);
            rightFlywheel.setVelocity(SHOOTER_VELOCITY);
            if (actionTimer.getElapsedTimeSeconds() > 1.5) {
                middleMotor.setPower(1.0);
                blockerServo.setPosition(0.75);
                intakeMotor.setPower(0);
            }
        } else {
            leftFlywheel.setVelocity(0);
            rightFlywheel.setVelocity(0);
            middleMotor.setPower(0);
            blockerServo.setPosition(1.0);
        }
    }

    public void runIntakeLogic(boolean active) {
        if (active) {
            intakeMotor.setPower(0.8);
            middleMotor.setPower(0.3);
            blockerServo.setPosition(1.0);
        } else {
            intakeMotor.setPower(0);
            middleMotor.setPower(0);
        }
    }
}