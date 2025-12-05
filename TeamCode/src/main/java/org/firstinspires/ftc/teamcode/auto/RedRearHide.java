package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@Autonomous(name = "Red Rear Hide", group = "auto")
public class RedRearHide extends AutoExample {
    private List < String > motif = new ArrayList < > (Arrays.asList("Purple", "Purple", "Green"));
    //                          red
    private final Pose startPose = new Pose(96, 9, Math.toRadians(90));
    private final Pose finalPose = new Pose(108, 10, Math.toRadians(270));
    private Pose currentPose;
    private Path toPark;
    public void buildPaths() {

        toPark = new Path(new BezierLine(startPose, finalPose));
        toPark.setLinearHeadingInterpolation(startPose.getHeading(), finalPose.getHeading());
    }

    public void runOpMode() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        waitForStart();
        initHardware();
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(new Pose());
        opmodeTimer.resetTimer();
        setPathState(0);

        while (opModeIsActive()) {
            // These loop the movements of the robot, these must be called continuously in order to work
            follower.update();
            switch (pathState) {
                case 0:
                    follower.followPath(toPark);
                    if (!follower.isBusy()) {
                        setPathState(1);
                        scoringState = 0;
                        actionTimer.resetTimer();
                    }
                    break;
                case 1:
                    follower.holdPoint(finalPose);
                    break;

            }
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
