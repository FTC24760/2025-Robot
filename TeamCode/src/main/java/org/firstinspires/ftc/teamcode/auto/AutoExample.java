package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.teleop.NewPrototypeTeleop;
import org.firstinspires.ftc.teamcode.teleop.oldrobot.OldPrototypeTeleop;

@Disabled
@Autonomous(name = "Rear double score", group = "auto")
public class AutoExample extends NewPrototypeTeleop {
    Follower follower;
    // Pedropathing variables;

    public Timer pathTimer, actionTimer, opmodeTimer;

    public int pathState;
    public int scoringState;
    public void runScoreLogic(boolean highSpeed) {

        switch (scoringState) {
            case 0:
                actionTimer.resetTimer();
                scoringState = 1;

                break;
            case 1:
                leftFlywheel.setPower(CLOSE_SHOOTER_VELOCITY);
                rightFlywheel.setPower(CLOSE_SHOOTER_VELOCITY);
                if (actionTimer.getElapsedTime() > 2000) {
                    blockerServo.setPosition(0.75);
                    scoringState = 2;
                    actionTimer.resetTimer();
                }

                break;
            case 2:
                middleMotor.setPower(1.0);
                intakeMotor.setPower(1.0);
                if (actionTimer.getElapsedTime() > 8000) {
                    scoringState = -1;
                }

                break;

        }
    }
}

