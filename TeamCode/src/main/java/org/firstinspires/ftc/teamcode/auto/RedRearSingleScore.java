package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;


@Autonomous(name = "Red Rear Single score", group = "auto")
public class RedRearSingleScore extends AutoExample {
    public Follower follower;
    private List < String > motif = new ArrayList < > (Arrays.asList("Purple", "Purple", "Green"));
    //                          red
    private static final Pose startPose = new Pose(96, 9, Math.toRadians(270));
    private static final Pose scorePose = new Pose(84, 84, Math.toRadians(225));
    private static final Pose parkPose = new Pose(96, 60, Math.toRadians(90));
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
        Paths paths = new Paths(follower);
        telemetry.addData("Direction", "Intake Back");
        telemetry.update();
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
                    follower.followPath(Paths.Path1);
                    if (follower.getPose().getY() > 82) {
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
