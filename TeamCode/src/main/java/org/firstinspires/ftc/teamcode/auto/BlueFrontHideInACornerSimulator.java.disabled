package org.firstinspires.ftc.teamcode.auto;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@Autonomous(name = "Blue Front Hide", group = "auto")
public class BlueFrontHideInACornerSimulator extends AutoExample {
   public static class Paths {
        public static Pose startPose = new Pose(144-122, 122, Math.toRadians(180-225));
        public static Pose endPose = new Pose(144-96, 132, Math.toRadians(180-90));

        public static Path path;
        public Paths(Follower follower) {
            path = new Path(new BezierLine(startPose, endPose));
            path.setLinearHeadingInterpolation(startPose.getHeading(), endPose.getHeading());
        }
    }
    public void runOpMode() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        actionTimer = new Timer();

        waitForStart();
        initHardware();
        initLogic();
        actionTimer.resetTimer();
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(Paths.startPose);
        opmodeTimer.resetTimer();
        setPathState(0);
        Paths paths = new Paths(follower);
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
