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

@Autonomous(name = "Red Back Hide", group = "Auto")
public class RedBackHide extends AutoExample {
    double flyWheelTargetSpeed = FAST_SHOOTER_VELOCITY;
    public Pose startPose = new Pose(84, 4, Math.toRadians(90));
    public Pose scorePose = new Pose(84.000, 12, Math.toRadians(66));

    public  Pose intake1Pose = new Pose(85, 36, Math.toRadians(0));
    public  Pose intake1GrabPose = new Pose(126, 36, Math.toRadians(0));

    public  Pose intake2Pose = new Pose(85, 60, Math.toRadians(0));
    public  Pose intake2GrabPose = new Pose(126, 60, Math.toRadians(0));

    public  Pose intake3Pose = new Pose(85, 84, Math.toRadians(0));
    public  Pose intake3GrabPose = new Pose(126, 84, Math.toRadians(0));

    public  Pose parkPose = new Pose(108, 12, Math.toRadians(90));

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
                shootingLogic(false, flyWheelTargetSpeed); // Spin up flywheels
                if (actionTimer.getElapsedTimeSeconds() > 2) intakeLogic();
                if (actionTimer.getElapsedTimeSeconds() > 2.5) {
                    actionTimer.resetTimer();
                    pathState = 3;
                }
                break;
            case 3:
                tripleShoot(actionTimer, flyWheelTargetSpeed); // Score
                if (actionTimer.getElapsedTimeSeconds() > 2.2) {
                    follower.followPath(myPaths.PathPark);
                    actionTimer.resetTimer();
                    pathState = 4;
                }
                break;
            case 8:
                tripleShoot(actionTimer, flyWheelTargetSpeed); // Score
                if (actionTimer.getElapsedTimeSeconds() > 2.2) {
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