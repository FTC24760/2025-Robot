package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Blue Front Score", group = "Auto")
public class BlueFrontScoring extends RedFrontScoring {
    public BlueFrontScoring() {
        startPose = new Pose(144 - 122, 122, Math.toRadians(180 - 45));
        scorePose = new Pose(144 - 84.000, 84.000, Math.toRadians(180 - 45));

        intake1Pose = new Pose(144 - 85, 84, Math.toRadians(180 - 0));
        intake1GrabPose = new Pose(144 - 116, 84, Math.toRadians(180 - 0));

        intake2Pose = new Pose(144 - 85, 60, Math.toRadians(180 - 0));
        intake2GrabPose = new Pose(144 - 118, 60, Math.toRadians(180 - 0));

        intake3Pose = new Pose(144 - 85, 36, Math.toRadians(180 - 0));
        intake3GrabPose = new Pose(144 - 120, 36, Math.toRadians(180 - 0));

        parkPose = new Pose(144 - 96, 60, Math.toRadians(180 - 90));
    }
}