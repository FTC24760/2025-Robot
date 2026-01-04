package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.SwerveRotationalPID.R_D;
import static org.firstinspires.ftc.teamcode.SwerveRotationalPID.R_I;
import static org.firstinspires.ftc.teamcode.SwerveRotationalPID.R_P;
import static java.lang.Math.abs;
import static java.lang.Math.atan2;
import static java.lang.Math.cos;
import static java.lang.Math.hypot;
import static java.lang.Math.sin;
import static java.lang.Math.toRadians;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


public class Swerve {
    private double MOTOR_REDUCTION = 28 * 2.89  * 3.61;
    private double SWERVE_REDUCTION = 6.25;
    private double SPEED_COEFFICIENT = 1;
    DcMotor drive1, drive2;
    double theta;
    public double targetAngle;
    public double wheelAngle;
    double wheelSpeed;
    double integralRotE;
    double derivativeRotE;
    double lastE = 0;
    double lastWheelRotation, lastWheelSpeed, speedE, integralSpeedE, derivativeSpeedE = 0;
    double lastTime;
    ElapsedTime timePID = new ElapsedTime();
    public Swerve(DcMotor drive1, DcMotor drive2, double theta) {//both motors going forwards will rotate the wheel counterclockwise
        this.drive1 = drive1;//going forwards makes the wheel go forwards
        this.drive2 = drive2;//going forwards makes the wheel go backwards
        this.theta = theta;
        lastTime = timePID.milliseconds();
        drive1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }
    void reset() {
        drive1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        integralRotE = 0;
        lastE = 0;
        drive1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void update(double rotatedX, double rotatedY, double driveTurn) {
        // -------------------------------- KINEMATICS --------------------------------
        double deltaX = rotatedX + driveTurn * cos(theta + toRadians(90)); // direction of rotation is 90 degrees from direction to center
        double deltaY = rotatedY + driveTurn * sin(theta + toRadians(90));
        double targetSpeed = hypot(deltaY, deltaX) * SPEED_COEFFICIENT;
        //targetSpeed = targetSpeed + (1-abs(targetSpeed))
        wheelAngle = a360((drive1.getCurrentPosition() + drive2.getCurrentPosition()) * 0.5
                / MOTOR_REDUCTION / SWERVE_REDUCTION * toRadians(360));

        if (targetSpeed > 0.01)
            targetAngle = a360(atan2(deltaY, deltaX) - toRadians(90));
        else
            targetAngle = theta;

        double rotE = a180(targetAngle - wheelAngle);
        targetSpeed *= cos(rotE); // cosine correction

        if (abs(rotE) > toRadians(90))
            rotE = a180(rotE + toRadians(180));

        // -------------------------------- PID --------------------------------
        integralRotE += rotE;
        derivativeRotE = (rotE - lastE)/(timePID.milliseconds() - lastTime);

        double rotationPower = (R_P * rotE) + (R_I * integralRotE) + (R_D *derivativeRotE);

            /*double wheelRotation = (drive1.getCurrentPosition() - drive2.getCurrentPosition())
                    / MOTOR_REDUCTION * toRadians(360);
            wheelSpeed = (wheelRotation - lastWheelRotation) / (runtime.milliseconds() - lastTime);
            speedE = targetSpeed - wheelSpeed;

            integralSpeedE += speedE;
            derivativeSpeedE = (wheelSpeed - lastWheelSpeed) / (runtime.milliseconds() - lastTime);
            double translationPower = (T_P * speedE) + (T_I * integralSpeedE) + (T_D * derivativeSpeedE);

            if (abs(rotationPower) + abs(translationPower) > 1)
                translationPower *= (1 - abs(rotationPower)) / abs(translationPower);*/
        double translationPower = targetSpeed;
        drive1.setPower(rotationPower + translationPower);
        drive2.setPower(rotationPower - translationPower);

        lastE = rotE;
        lastTime = timePID.milliseconds();
        //lastWheelRotation = wheelRotation;
        //lastWheelSpeed = wheelSpeed;

    }
    double a360(double angle) {
        while (angle > toRadians(360)) angle -= toRadians(360);
        while (angle < 0) angle += toRadians(360);
        return angle;
    }
    double a180(double angle) {
        while (angle > toRadians(180)) angle -= toRadians(360);
        while (angle < -180) angle += toRadians(360);
        return angle;
    }
}