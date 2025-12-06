package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Red Single Double", group = "Auto")
public class BlueFrontScoringSingle extends AutoExample {
    public static Pose startPose = new Pose(144-122, 122, Math.toRadians(180-225));
    public static Pose scorePose = new Pose(144-84.000, 84.000, Math.toRadians(180-225));
    public static Pose parkPose = new Pose(144-96, 108, Math.toRadians(180-90));

    public static class Paths {
        public static Path Path1, Path2, Path3, Path4, Path5, Path6, Path7;

        public Paths(Follower follower) {
            Path1 = new Path(new BezierLine(startPose, scorePose));
            Path1.setLinearHeadingInterpolation(scorePose.getHeading(), scorePose.getHeading());

            Path7 = new Path(new BezierLine(scorePose, parkPose));
            Path7.setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading());
        }
    }
    public void runOpMode() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        actionTimer = new Timer();
        RedFrontScoringSingle.Paths paths = new RedFrontScoringSingle.Paths(follower);
        waitForStart();
        initHardware();
        initLogic();
        actionTimer.resetTimer();
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        opmodeTimer.resetTimer();
        setPathState(0);

        for (int i = 0; i < 3; i++) {
            slots.get(i).isClawOpen = false;
        }
        updateRevolverServos();

        while (opModeIsActive()) {
            // These loop the movements of the robot, these must be called continuously in order to work
            follower.update();
            switch (pathState) {
                case 0:
                    follower.followPath(RedFrontScoringSingle.Paths.Path1);
                    if (follower.getPose().getY() < 86) {
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
                    follower.followPath(RedFrontScoringSingle.Paths.Path7);
                    if (!follower.isBusy()) {
                        setPathState(8);
                        actionTimer.resetTimer();
                    }
                    break;
                case 8:
                    //follower.holdPoint(parkPose);
                    break;
            }
            updateRevolverServos();
            //follower.setPose(getRobotPoseFromCamera());
            // Feedback to Driver Hub for debugging
            telemetry.addData("path state", pathState);
            telemetry.addData("x", follower.getPose().getX());
            telemetry.addData("y", follower.getPose().getY());
            telemetry.addData("heading", follower.getPose().getHeading());
            telemetry.update();
        }
    }
}
