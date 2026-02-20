package org.firstinspires.ftc.teamcode.teleop;

import static java.lang.Math.toRadians;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Pedro TeleOp Blue")
public class PedroTeleopBlue extends PedroTeleopRed {
    public static final Pose frontScorePose = new Pose(144-84, 84, toRadians(180-45));
    public static final Pose backScorePose = new Pose(144-84, 12, toRadians(180-68.2));
    public static final Pose frontAutoParkPose = new Pose(144-96, 60, toRadians(180-90));
    public static final Pose backAutoParkPose = new Pose(144-108, 12, toRadians(180-90));
}
