package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Blue Front Double", group = "Auto")
public class BlueFrontScoring extends AutoExample {
    public static Pose scorePose = new Pose(84.000, 84.000, Math.toRadians(225));
    public static Pose parkPose = new Pose(84, 60, Math.toRadians(315));
    public static class Paths {
        public static Path Path1, Path2, Path3, Path4, Path5, Path6, Path7;

        public Paths(Follower follower) {
            Path1 = new Path(new BezierLine(new Pose(144-122.000, 122.000), new Pose(144-84.000, 84.000)));
            Path1.setLinearHeadingInterpolation(Math.toRadians(180-225), Math.toRadians(180-225));

            Path2 = new Path(new BezierLine(new Pose(144-84.000, 84.000), new Pose(144-102.000, 84.000)));
            Path2.setLinearHeadingInterpolation(Math.toRadians(180-225), Math.toRadians(180-0));

            Path3 = new Path(new BezierLine(new Pose(144-102.000, 84.000), new Pose(144-108.000, 84.000)));
            Path3.setConstantHeadingInterpolation(Math.toRadians(180-0));

            Path4 = new Path(new BezierLine(new Pose(144-108.000, 84.000), new Pose(144-112.000, 84.000)));
            Path4.setConstantHeadingInterpolation(Math.toRadians(180-0));

            Path5 = new Path(new BezierLine(new Pose(144-112.000, 84.000), new Pose(144-118.000, 84.000)));
            Path5.setConstantHeadingInterpolation(Math.toRadians(180-0));

            Path6 = new Path(new BezierLine(new Pose(144-118.000, 84.000), new Pose(144-84.000, 84.000)));
            Path6.setLinearHeadingInterpolation(Math.toRadians(180-0), Math.toRadians(180-225));

            Path7 = new Path(new BezierLine(new Pose(144-84.000, 84.000), new Pose(144-84.000, 60.000)));
            Path7.setLinearHeadingInterpolation(Math.toRadians(180-225), Math.toRadians(180-315));
        }
    }
    public void runOpMode() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        Paths paths = new Paths(follower);
        waitForStart();
        initHardware();
        initLogic();
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
                        setPathState(2);
                        targetSlotIndex = getNextEmptySlot();
                        startIntake();
                    }
                    break;
                case 2:
                    follower.followPath(Paths.Path2);
                    intakeSpinner.setPower(1);
                    if (!follower.isBusy()) {
                        slots.get(targetSlotIndex).isClawOpen = false;
                        slots.get(targetSlotIndex).color = "Purple";
                        slots.get(targetSlotIndex).occupied = true;

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
                        slots.get(targetSlotIndex).color = "Green";
                        slots.get(targetSlotIndex).occupied = true;

                        intakeSpinner.setPower(0);

                        setPathState(5);
                        actionTimer.resetTimer();
                    }
                    break;
                case 5:
                    follower.followPath(Paths.Path5);
                    if (!follower.isBusy()) {
                        setPathState(6);
                        actionTimer.resetTimer();
                    }
                case 6:
                    follower.holdPoint(scorePose);
                    runScoreLogic(false);
                    if (scoringState == -1) {
                        setPathState(7);
                        targetSlotIndex = getNextEmptySlot();
                        startIntake();
                    }
                    break;
                case 7:
                    follower.followPath(Paths.Path6);
                    if (!follower.isBusy()) {
                        setPathState(8);
                        actionTimer.resetTimer();
                    }
                    break;
                case 8:
                    follower.holdPoint(parkPose);
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
