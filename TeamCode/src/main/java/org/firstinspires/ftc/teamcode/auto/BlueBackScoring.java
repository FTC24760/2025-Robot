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

@Autonomous(name = "Blue Back Score", group = "Auto")
public class BlueBackScoring extends AutoExample {
    double flyWheelTargetSpeed = FAST_SHOOTER_VELOCITY;
    Paths paths;
    public static Pose startPose = new Pose(144-84, 8, Math.toRadians(180-90));
    public static Pose scorePose = new Pose(144-84.000, 12, Math.toRadians(180-58));

    public static Pose intake3Pose = new Pose(144-85, 84, Math.toRadians(180-0));
    public static Pose intake3GrabPose = new Pose(144-116, 84, Math.toRadians(180-0));

    public static Pose intake2Pose = new Pose(144-85, 60, Math.toRadians(180-0));
    public static Pose intake2GrabPose = new Pose(144-118, 60, Math.toRadians(180-0));

    public static Pose intake1Pose = new Pose(144-85, 36, Math.toRadians(180-0));
    public static Pose intake1GrabPose = new Pose(144-120, 36, Math.toRadians(180-0));

    public static Pose parkPose = new Pose(144-108, 12, Math.toRadians(180-90));

    public static double SHOOTING_TIME = 2.0; // time when the shooter is on PLUS the wait time

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
    @Override
    public void init() {
        super.startPose = this.startPose;
        super.init();
        paths = new Paths(follower);
    }

    @Override
    public void pathLogic() {
        switch (pathState) {
            case 0:
                follower.followPath(Paths.Path1);
                pathState = 1;
                break;
            case 1:
                if (isAtPose(scorePose) || !follower.isBusy()) {
                    follower.holdPoint(scorePose);
                    actionTimer.resetTimer();
                    pathState = 2;
                }
                break;
            case 2:
                shootingLogic(true, flyWheelTargetSpeed); // Spin up flywheels
                if (actionTimer.getElapsedTimeSeconds() > 2.0) {
                    actionTimer.resetTimer();
                    pathState = 3;
                }
                break;
            case 3:
                shootingLogic(true, flyWheelTargetSpeed); // Score
                if (actionTimer.getElapsedTimeSeconds() > SHOOTING_TIME) {
                    follower.followPath(Paths.PathToIntake1);
                    actionTimer.resetTimer();
                    pathState = 4;
                }
                break;
            case 4:
                double headingError = Math.abs(follower.getPose().getHeading() - intake1Pose.getHeading());
                if (headingError < 0.07 || !follower.isBusy()) {
                    follower.followPath(Paths.PathGrab1);
                    actionTimer.resetTimer();
                    pathState = 5;
                }
                break;
            case 5:
                intakeLogic();

                if (isAtPose(intake1GrabPose) || !follower.isBusy()) {
                    follower.followPath(Paths.PathScore1);
                    actionTimer.resetTimer();
                    pathState = 6;
                }
                break;
            case 6:
                if (actionTimer.getElapsedTimeSeconds() < 0.5)
                    intakeLogic();
                if (isAtPose(scorePose) || !follower.isBusy()) {
                    follower.holdPoint(scorePose);
                    actionTimer.resetTimer();
                    pathState = 7;
                }
                break;
            case 7:
                shootingLogic(true, flyWheelTargetSpeed); // Spin up flywheels
                if (actionTimer.getElapsedTimeSeconds() > 2.0) {
                    actionTimer.resetTimer();
                    pathState = 8;
                }
                break;
            case 8:
                shootingLogic(true, flyWheelTargetSpeed); // Score
                if (actionTimer.getElapsedTimeSeconds() > SHOOTING_TIME) {
                    follower.followPath(Paths.PathToIntake2);
                    actionTimer.resetTimer();
                    pathState = 9;
                }
                break;
            case 9:
                if (isAtPose(intake2Pose) || !follower.isBusy()) {
                    follower.followPath(Paths.PathGrab2);
                    actionTimer.resetTimer();
                    pathState = 10;
                }
                break;
            case 10:
                intakeLogic();

                if (isAtPose(intake2GrabPose) || !follower.isBusy()) {
                    follower.followPath(Paths.PathScore2);
                    pathState = 11;
                    actionTimer.resetTimer();
                }
                break;
            case 11:

                if (actionTimer.getElapsedTimeSeconds() < 0.5)
                    intakeLogic();
                if (isAtPose(scorePose) || !follower.isBusy()) {
                    follower.holdPoint(scorePose);
                    actionTimer.resetTimer();
                    pathState = 12;
                }
                break;
            case 12:
                shootingLogic(true, flyWheelTargetSpeed); // Spin up flywheels
                if (actionTimer.getElapsedTimeSeconds() > 2.0) {
                    actionTimer.resetTimer();
                    pathState = 13;
                }
                break;
            case 13:
                shootingLogic(true, flyWheelTargetSpeed); // Score
                if (actionTimer.getElapsedTimeSeconds() > SHOOTING_TIME) {
                    follower.followPath(Paths.PathToIntake3);
                    pathState = 14;
                }
                break;
            case 14:
                if (isAtPose(intake3Pose) || !follower.isBusy()) {
                    follower.followPath(Paths.PathGrab3);
                    pathState = 15;
                }
                break;
            case 15:
                intakeLogic();

                if (isAtPose(intake3GrabPose) || !follower.isBusy()) {
                    follower.followPath(Paths.PathScore3);
                    actionTimer.resetTimer();
                    pathState = 16;
                }
                break;
            case 16:
                if (actionTimer.getElapsedTimeSeconds() < 0.5)
                    intakeLogic();
                if (isAtPose(scorePose) || !follower.isBusy()) {
                    follower.holdPoint(scorePose);
                    actionTimer.resetTimer();
                    pathState = 17;
                }
                break;
            case 17:
                shootingLogic(true, flyWheelTargetSpeed); // Spin up flywheels
                if (actionTimer.getElapsedTimeSeconds() > 2.0) {
                    actionTimer.resetTimer();
                    pathState = 18;
                }
                break;
            case 18:
                shootingLogic(true, flyWheelTargetSpeed); // Score
                if (actionTimer.getElapsedTimeSeconds() > 3) {
                    follower.followPath(Paths.PathPark);
                    actionTimer.resetTimer();
                    pathState = 19;
                }
                break;
            case 19:
                if (!follower.isBusy() || actionTimer.getElapsedTimeSeconds() > 5) {
                    follower.holdPoint(parkPose);
                    pathState = 20;
                }
                break;
            case 20:
                // Auto sequence finished
                break;
        }
    }
}