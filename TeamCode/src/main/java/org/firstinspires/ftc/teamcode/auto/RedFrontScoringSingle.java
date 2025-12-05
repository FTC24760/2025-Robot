package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Red Single Double", group = "Auto")
public class RedFrontScoringSingle extends AutoExample {

    public static Pose scorePose = new Pose(84.000, 84.000, Math.toRadians(225));
    public static Pose parkPose = new Pose(84, 60, Math.toRadians(315));
    public static class Paths {
        public static Path Path1, Path2, Path3, Path4, Path5, Path6, Path7;

        public Paths(Follower follower) {
            Path1 = new Path(new BezierLine(new Pose(122.000, 122.000), new Pose(84.000, 84.000)));
            Path1.setLinearHeadingInterpolation(Math.toRadians(225), Math.toRadians(225));

            Path7 = new Path(new BezierLine(new Pose(84.000, 84.000), new Pose(84.000, 60.000)));
            Path7.setLinearHeadingInterpolation(Math.toRadians(225), Math.toRadians(90));
        }
    }
    public void runOpMode() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        waitForStart();
        initHardware();
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose());
        opmodeTimer.resetTimer();
        setPathState(0);

        while (opModeIsActive()) {
            // These loop the movements of the robot, these must be called continuously in order to work
            follower.update();
            switch (pathState) {
                case 0:
                    follower.followPath(Paths.Path1);
                    if (!follower.isBusy()) {
                        setPathState(1);
                        scoringState = 0;
                        actionTimer.resetTimer();
                    }
                    break;
                case 1:
                    follower.holdPoint(scorePose);
                    runScoreLogic(false);
                    if (scoringState == -1) {
                        setPathState(7);
                        targetSlotIndex = getNextEmptySlot();
                        startIntake();
                    }
                    break;
                case 7:
                    follower.followPath(Paths.Path7);
                    if (!follower.isBusy()) {
                        setPathState(8);
                        actionTimer.resetTimer();
                    }
                    break;
                case 8:
                    follower.holdPoint(parkPose);
            }
            updateRevolverServos();
            follower.setPose(getRobotPoseFromCamera());
            // Feedback to Driver Hub for debugging
            telemetry.addData("path state", pathState);
            telemetry.addData("x", follower.getPose().getX());
            telemetry.addData("y", follower.getPose().getY());
            telemetry.addData("heading", follower.getPose().getHeading());
            telemetry.update();
        }
    }
}
