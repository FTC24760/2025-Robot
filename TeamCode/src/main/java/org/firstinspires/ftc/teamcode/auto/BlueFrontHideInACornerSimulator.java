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
    public static Pose startPose = new Pose(122, 122, Math.toRadians(45));
    public static Pose parkPose = new Pose(96, 60, Math.toRadians(90));
    Paths paths;

    public static class Paths {
        public static Path Path1, Path2, Path3, Path4, Path5, Path6, Path7;

        public Paths(Follower follower) {
            Path1 = new Path(new BezierLine(startPose, parkPose));
            Path1.setLinearHeadingInterpolation(startPose.getHeading(), parkPose.getHeading());

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
        switch(pathState) {
            case 0:
                follower.followPath(Paths.Path1);
                pathState = 1;
                break;
            case 1:
                if (!follower.isBusy()) {
                    follower.holdPoint(parkPose);
                    pathState = 2;
                }
                break;
            case 2:
                // Finished
                break;
        }
    }
}