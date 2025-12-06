package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@Autonomous(name = "Red Rear double score", group = "auto")
public class RedRearDoubleScore extends AutoExample {
    public Follower follower;
    private List < String > motif = new ArrayList < > (Arrays.asList("Purple", "Purple", "Green"));
    //                          red
    private final Pose startPose = new Pose(84, 9, Math.toRadians(270));
    private final Pose scorePose = new Pose(84, 12, Math.toRadians(249.67));
    private static final Pose finalPose = new Pose(108, 10, Math.toRadians(270));
    public static class Paths {

        public static PathChain Path1;
        public static PathChain Path2;
        public static PathChain Path3;
        public static PathChain Path4;
        public static PathChain Path5;
        public static PathChain Path6;
        public static PathChain Path7;

        public Paths(Follower follower) {
            Path1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(84.000, 9.000), new Pose(84.000, 12.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(180+69.67))
                    .build();

            Path2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(84.000, 12.000), new Pose(104.000, 36.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(249.67), Math.toRadians(0))
                    .build();

            Path3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(104.000, 36.000), new Pose(108.000, 36.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            Path4 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(108.000, 36.000), new Pose(112.000, 36.000))
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            Path5 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(112.000, 36.000), new Pose(118.000, 36.000))
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            Path6 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(118.000, 36.000), new Pose(84.000, 12.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(249.67))
                    .build();
            Path7 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(84.000, 12.000), finalPose)
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(249.67), Math.toRadians(90))
                    .build();
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
            while (opModeIsActive()) {
                // These loop the movements of the robot, these must be called continuously in order to work
                follower.update();
                switch (pathState) {
                    case 0:
                        follower.followPath(RedFrontScoring.Paths.Path1);
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
                            setPathState(2);
                            targetSlotIndex = getNextEmptySlot();
                            startIntake();
                        }
                        break;
                    case 2:
                        follower.followPath(Paths.Path2);
                        intakeSpinner.setPower(1);
                        if (!follower.isBusy()) {
                            setPathState(3);
                            actionTimer.resetTimer();
                        }
                    case 3:
                        follower.followPath(Paths.Path3);
                        if (actionTimer.getElapsedTimeSeconds() > 0.1) {
                            targetSlotIndex = getNextEmptySlot();
                            startIntake();
                        }
                        if (!follower.isBusy()) {
                            slots.get(targetSlotIndex).isClawOpen = false;
                            slots.get(targetSlotIndex).color = "Purple";
                            slots.get(targetSlotIndex).occupied = true;

                            setPathState(4);
                            actionTimer.resetTimer();
                        }
                        break;
                    case 4:
                        follower.followPath(Paths.Path4);
                        if (actionTimer.getElapsedTimeSeconds() > 0.1) {
                            targetSlotIndex = getNextEmptySlot();
                            startIntake();
                        }
                        if (!follower.isBusy()) {
                            slots.get(targetSlotIndex).isClawOpen = false;
                            slots.get(targetSlotIndex).color = "Purple";
                            slots.get(targetSlotIndex).occupied = true;


                            setPathState(5);
                            actionTimer.resetTimer();
                        }
                        break;
                    case 5:
                        follower.followPath(Paths.Path5);
                        if (actionTimer.getElapsedTimeSeconds() > 0.1) {
                            targetSlotIndex = getNextEmptySlot();
                            startIntake();
                        }

                        if (!follower.isBusy()) {
                            slots.get(targetSlotIndex).isClawOpen = false;
                            slots.get(targetSlotIndex).color = "Green";
                            slots.get(targetSlotIndex).occupied = true;

                            setPathState(6);
                            actionTimer.resetTimer();
                        }
                    case 6:
                        follower.followPath(Paths.Path6);
                        if (!follower.isBusy()) {
                            setPathState(7);
                            actionTimer.resetTimer();
                        }
                    case 7:
                        follower.holdPoint(scorePose);
                        runScoreLogic(false);
                        if (scoringState == -1) {
                            setPathState(8);
                            targetSlotIndex = getNextEmptySlot();
                            startIntake();
                        }
                        break;
                    case 8:
                        follower.followPath(RedFrontScoring.Paths.Path7);
                        if (!follower.isBusy()) {
                            setPathState(9);
                            actionTimer.resetTimer();
                        }
                        break;
                    case 9:
                        follower.holdPoint(finalPose);
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
}
