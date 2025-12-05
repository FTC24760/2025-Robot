package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Blue Front Hide", group = "auto")
public class BlueFrontHideInACornerSimulator extends AutoExample {
    public static class Paths {
        public static Pose startPose = new Pose(122, 122, Math.toRadians(37));
        public static Pose endPose = new Pose(84, 132, Math.toRadians(0));
        public static Path path;

        public Paths(Follower follower) {
            path = new Path(new BezierLine(startPose, endPose));
            path.setLinearHeadingInterpolation(startPose.getHeading(), endPose.getHeading());
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
                    follower.followPath(Paths.path);
                    if (!follower.isBusy()) {
                        setPathState(1);
                        scoringState = 0;
                        actionTimer.resetTimer();
                    }
                    break;
                case 1:
                    follower.holdPoint(Paths.endPose);
                    break;

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
