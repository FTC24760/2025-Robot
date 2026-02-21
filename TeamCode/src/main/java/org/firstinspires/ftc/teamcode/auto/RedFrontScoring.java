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

@Autonomous(name = "Red Front Score", group = "Auto")
public class RedFrontScoring extends AutoExample {
    public Pose startPose = new Pose(122, 122, Math.toRadians(45));
    public Pose scorePose = new Pose(84.000, 84.000, Math.toRadians(45));

    public  Pose intake1Pose = new Pose(85, 84, Math.toRadians(0));
    public  Pose intake1GrabPose = new Pose(116, 84, Math.toRadians(0));

    public  Pose intake2Pose = new Pose(85, 60, Math.toRadians(0));
    public  Pose intake2GrabPose = new Pose(118, 60, Math.toRadians(0));

    public  Pose intake3Pose = new Pose(85, 36, Math.toRadians(0));
    public  Pose intake3GrabPose = new Pose(120, 36, Math.toRadians(0));

    public  Pose parkPose = new Pose(96, 60, Math.toRadians(90));

    public Paths myPaths;

    public  class Paths {
        public  Path Path1, PathToIntake1, PathGrab1, PathScore1;
        public  Path PathToIntake2, PathGrab2, PathScore2;
        public  Path PathToIntake3, PathGrab3, PathScore3;
        public  Path PathPark;

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
        myPaths = new Paths(follower);
    }

    @Override
    public void pathLogic() {
        switch (pathState) {
            case 0:
                follower.followPath(myPaths.Path1);
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
                shootingLogic(true);
                if (actionTimer.getElapsedTimeSeconds() > 3) {
                    follower.followPath(myPaths.PathToIntake1);
                    pathState = 3;
                }
                break;
            case 3:
                double headingError = Math.abs(follower.getPose().getHeading() - intake1Pose.getHeading());
                if (headingError < 0.07 || !follower.isBusy()) {
                    intakeLogic();
                    follower.followPath(myPaths.PathGrab1);
                    pathState = 4;
                }
                break;
            case 4:
                if (isAtPose(intake1GrabPose) || !follower.isBusy()) {
                    follower.followPath(myPaths.PathScore1);
                    pathState = 5;
                }
                break;
            case 5:
                if (isAtPose(scorePose) || !follower.isBusy()) {
                    follower.holdPoint(scorePose);
                    actionTimer.resetTimer();
                    pathState = 6;
                }
                break;
            case 6:
                shootingLogic(true);
                if (actionTimer.getElapsedTimeSeconds() > 3) {
                    follower.followPath(myPaths.PathToIntake2);
                    pathState = 7;
                }
                break;
            case 7:
                if (isAtPose(intake2Pose) || !follower.isBusy()) {
                    intakeLogic();
                    follower.followPath(myPaths.PathGrab2);
                    pathState = 8;
                }
                break;
            case 8:
                if (isAtPose(intake2GrabPose) || !follower.isBusy()) {
                    follower.followPath(myPaths.PathScore2);
                    pathState = 9;
                }
                break;
            case 9:
                if (isAtPose(scorePose) || !follower.isBusy()) {
                    follower.holdPoint(scorePose);
                    actionTimer.resetTimer();
                    pathState = 10;
                }
                break;
            case 10:
                shootingLogic(true);
                if (actionTimer.getElapsedTimeSeconds() > 3) {
                    follower.followPath(myPaths.PathToIntake3);
                    pathState = 11;
                }
                break;
            case 11:
                if (isAtPose(intake3Pose) || !follower.isBusy()) {
                    intakeLogic();
                    follower.followPath(myPaths.PathGrab3);
                    pathState = 12;
                }
                break;
            case 12:
                if (isAtPose(intake3GrabPose) || !follower.isBusy()) {
                    follower.followPath(myPaths.PathScore3);
                    pathState = 13;
                }
                break;
            case 13:
                if (isAtPose(scorePose) || !follower.isBusy()) {
                    follower.holdPoint(scorePose);
                    actionTimer.resetTimer();
                    pathState = 14;
                }
                break;
            case 14:
                shootingLogic(true);
                if (actionTimer.getElapsedTimeSeconds() > 3) {
                    follower.followPath(myPaths.PathPark);
                    actionTimer.resetTimer();
                    pathState = 15;
                }
                break;
            case 15:
                if (!follower.isBusy() || actionTimer.getElapsedTimeSeconds() > 5) {
                    follower.holdPoint(parkPose);
                    pathState = 16;
                }
                break;
            case 16:
                // Auto sequence finished
                break;
        }
    }
}