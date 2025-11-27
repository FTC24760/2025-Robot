package org.firstinspires.ftc.teamcode.auto; // make sure this aligns with class location

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Example Auto", group = "Examples")
public class AutoTest extends OpMode {
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private enum RobotState {
        INITIAL,
        SEARCHING,
        SCORING
    }

    private RobotState pathState;
    // --------------------------------
    //                          red
    // idk
    private final Pose startPose = new Pose(96, 9, Math.toRadians(90));
    private final Pose scorePose = new Pose(84, 84, Math.toRadians(45));
    private Path initialScoring;
    public void buildPaths() {
        initialScoring = new Path(new BezierLine(startPose, scorePose));
        initialScoring.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case INITIAL:
                follower.followPath(initialScoring);
                setPathState(RobotState.SCORING);
                break;
            case SEARCHING:
                break;
            case SCORING:

                break;

            /* You could check for
            - Follower State: "if(!follower.isBusy()) {}"
            - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
            - Robot Position: "if(follower.getPose().getX() > 36) {}"
            */

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                //if (!follower.isBusy()) {
                    /* Score Preload */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                   // follower.followPath(grabPickup1, true);
                   // setPathState(2);
              //  }
           //     break;
           // case 2:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
              //  if (!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
              //      follower.followPath(scorePickup1, true);
              //      setPathState(3);
             //   }
            //    break;
        }
    }

    /**
     * These change the states of the paths and actions. It will also reset the timers of the individual switches
     **/
    public void setPathState(RobotState pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(RobotState.INITIAL);
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();


        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);

    }

    @Override
    public void loop() {

        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        autonomousPathUpdate();

        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    @Override
    public void init_loop() {}

    @Override
    public void stop() {}
}