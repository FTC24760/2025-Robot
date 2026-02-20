package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.teleop.NewPrototypeTeleop;
import org.firstinspires.ftc.teamcode.teleop.NewPrototypeTeleopPedroPathingRed;
import org.firstinspires.ftc.teamcode.teleop.oldrobot.OldPrototypeTeleop;

@Disabled
@Autonomous(name = "Rear double score", group = "auto")
public class AutoExample extends NewPrototypeTeleopPedroPathingRed {
    public static double THRESHOLD = 1.0; // threshold for location thingy
    Follower follower;
    public Timer pathTimer, actionTimer, opmodeTimer;

    public int pathState;
    Pose startPose;
    public static class Paths {
        Paths(Follower follower) {
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

    }

    public boolean isAtPose(Pose target) {
        double distance = Math.hypot(
                follower.getPose().getX() - target.getX(),
                follower.getPose().getY() - target.getY()
        );
        return distance < THRESHOLD;
    }

    @Override
    public void init() {
        initHardware();
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        actionTimer = new Timer();
        Paths paths = new Paths(follower);
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
        resetMotors();
        pathLogic();
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }
    void pathLogic() {}

}

