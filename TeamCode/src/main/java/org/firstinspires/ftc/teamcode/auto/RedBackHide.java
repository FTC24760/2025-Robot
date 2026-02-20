package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Red Back Hide", group = "auto")
public class RedBackHide extends AutoExample {
    public static Pose startPose = new Pose(84, 8, Math.toRadians(90));
    public static Pose parkPose = new Pose(108, 12, Math.toRadians(90));
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
        follower.followPath(Paths.Path1);
    }
}
