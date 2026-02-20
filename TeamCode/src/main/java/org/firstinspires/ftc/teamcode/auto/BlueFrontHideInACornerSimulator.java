package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Blue Front Hide", group = "auto")
public class BlueFrontHideInACornerSimulator extends RedFrontHideInACornerSimulator {
    public static Pose startPose = new Pose(144-122, 122, Math.toRadians(180-45));
    public static Pose parkPose = new Pose(144-96, 60, Math.toRadians(180-90));


    public static class Paths {
        public static Path Path1;

        public Paths(Follower follower) {
            Path1 = new Path(new BezierLine(startPose, parkPose));
            Path1.setLinearHeadingInterpolation(startPose.getHeading(), parkPose.getHeading());

        }
    }
}
