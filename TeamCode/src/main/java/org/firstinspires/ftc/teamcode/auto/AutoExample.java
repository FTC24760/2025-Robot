package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.teleop.PedroTeleopRed;

@Disabled
@Autonomous(name = "Rear double score", group = "auto")
public class AutoExample extends PedroTeleopRed {
    public static double THRESHOLD = 1.0; // threshold for location thingy
    Follower follower;
    public Timer pathTimer, actionTimer, opmodeTimer;

    public int pathState;
    Pose startPose;

    public void initHardware() {
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");
        middleMotor = hardwareMap.get(DcMotorEx.class, "middle");
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        middleMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFlywheel = hardwareMap.get(DcMotorEx.class, "leftShooter");
        rightFlywheel = hardwareMap.get(DcMotorEx.class, "rightShooter");

        blocker = hardwareMap.get(Servo.class, "blockerL");
        blocker2 = hardwareMap.get(Servo.class, "blockerR");


        leftFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFlywheel.setDirection(DcMotorSimple.Direction.REVERSE);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(PIPELINE_NEURAL);
        limelight.start();

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
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        pathTimer = new Timer();
        opmodeTimer = new Timer();
        actionTimer = new Timer();
    }
    @Override
    public void start() {
        actionTimer.resetTimer();
        opmodeTimer.resetTimer();
        pathState = 0;
    }
    @Override
    public void loop() {
        follower.update();
        resetMotors();
        pathLogic();
        updateMotors();
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }
    void pathLogic() {}

}

