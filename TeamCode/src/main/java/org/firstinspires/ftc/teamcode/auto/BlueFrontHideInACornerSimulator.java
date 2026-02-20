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
    public static Pose startPose = new Pose(144-122, 122, Math.toRadians(180-45));
    public static Pose parkPose = new Pose(144-96, 60, Math.toRadians(180-90));


    public static class Paths {
        public static Path Path1, Path2, Path3, Path4, Path5, Path6, Path7;

        public Paths(Follower follower) {
            Path1 = new Path(new BezierLine(startPose, parkPose));
            Path1.setLinearHeadingInterpolation(startPose.getHeading(), parkPose.getHeading());

        }
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
        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        switch (pathState) {
            case 0:
                follower.followPath(Paths.Path1);

                break;
            case 1:
                break;
        }
        //follower.setPose(getRobotPoseFromCamera());
        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();


    }
}
