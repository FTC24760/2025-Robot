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

@Autonomous(name = "Red Back Hide", group = "Auto")
public class RedBackHide extends AutoExample {
    public static Pose startPose = new Pose(84, 8, Math.toRadians(90));


    public static Pose parkPose = new Pose(108, 12, Math.toRadians(90));

    public static double THRESHOLD = 1.0; // threshold for location thingy
    public static double SPIN_TIME = 0.9;
    public static double SHOOTING_TIME = 2.0; //  time when the shooter is on PLUs the wait time


    public DcMotorEx intakeMotor, middleMotor, leftFlywheel, rightFlywheel;
    public Servo hoodServo, blockerServo;

    public static class Paths {
        public static Path PathPark;

        public Paths(Follower follower) {

            PathPark = new Path(new BezierLine(startPose, parkPose));
            PathPark.setLinearHeadingInterpolation(startPose.getHeading(), parkPose.getHeading());
        }
    }

    public void initHardware() {
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");
        middleMotor = hardwareMap.get(DcMotorEx.class, "middle");
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        middleMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFlywheel = hardwareMap.get(DcMotorEx.class, "leftShooter");
        rightFlywheel = hardwareMap.get(DcMotorEx.class, "rightShooter");

        leftFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFlywheel.setDirection(DcMotorSimple.Direction.REVERSE);

        hoodServo = hardwareMap.get(Servo.class, "hood");
        blockerServo = hardwareMap.get(Servo.class, "blocker");
        blockerServo.setPosition(1.0);
    }

    public boolean isAtPose(Pose target) {
        double distance = Math.hypot(
                follower.getPose().getX() - target.getX(),
                follower.getPose().getY() - target.getY()
        );
        return distance < THRESHOLD;
    }
    Paths paths;
    @Override
    public void init() {
        initHardware();
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        actionTimer = new Timer();
        paths = new Paths(follower);
    }
    @Override
    public void start() {

        actionTimer.resetTimer();
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        opmodeTimer.resetTimer();
        pathState = 0;
    }

    @Override
    public void loop() {
        follower.update();
        follower.followPath(paths.PathPark);
        follower.update();
        switch (pathState) {
            case 0:
                follower.followPath(Paths.PathPark);
                if (pathTimer.getElapsedTime() > 5000)
                    pathState++;
                break;
            case 1:
                follower.holdPoint(parkPose);
                break;
        }
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    public void runScoreLogic(boolean active) {
        if (active) {
            leftFlywheel.setVelocity(FAR_SHOOTER_VELOCITY);
            rightFlywheel.setVelocity(FAR_SHOOTER_VELOCITY);
            if (actionTimer.getElapsedTimeSeconds() > SPIN_TIME) {
                middleMotor.setPower(1.0);
                blockerServo.setPosition(0.75);
                intakeMotor.setPower(0);
            }
        } else {
            leftFlywheel.setVelocity(0);
            rightFlywheel.setVelocity(0);
            middleMotor.setPower(0);
            blockerServo.setPosition(1.0);
        }
    }

    public void runIntakeLogic(boolean active) {
        if (active) {
            intakeMotor.setPower(0.8);
            middleMotor.setPower(0.3);
            blockerServo.setPosition(1.0);
        } else {
            intakeMotor.setPower(0);
            middleMotor.setPower(0);
        }
    }
}