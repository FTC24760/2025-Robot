/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.teleop.oldrobot;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name="Drive", group="Teleop")
public class Teleop extends OpMode
{
    String mode = "in";
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor driveLF, driveLB, driveRF, driveRB;
    private Servo revolver;
    private Servo indicatorLight1, indicatorLight2, indicatorLight3;
    private Servo indicatorLights[] = {indicatorLight1, indicatorLight2, indicatorLight3};
    char clawBallColor[] = {'N', 'N', 'N'};
    IMU imu;
    int revolverPosition;
    @Override
    public void init() {
        driveLF = hardwareMap.get(DcMotor.class, "flDrive");
        driveLB = hardwareMap.get(DcMotor.class, "rlDrive");
        driveRF = hardwareMap.get(DcMotor.class, "frDrive");
        driveRB = hardwareMap.get(DcMotor.class, "rrDrive");
        driveLF.setDirection(DcMotor.Direction.REVERSE);
        driveLB.setDirection(DcMotor.Direction.REVERSE);
        driveRF.setDirection(DcMotor.Direction.FORWARD);
        driveRB.setDirection(DcMotor.Direction.FORWARD);
        // Retrieve the IMU from the hardware map
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }
    
    @Override
    public void init_loop() {
    }
    
    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {
        double powerLF, powerLB, powerRF, powerRB;
        if (gamepad1.left_bumper) {
            imu.resetYaw();
        }

        // drive
        double botHeading = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double driveY    = -gamepad1.left_stick_y;
        double driveX    = gamepad1.left_stick_x;
        double driveTurn = gamepad1.right_stick_x;
        double rotatedX = driveX * Math.cos(botHeading) - driveY * Math.sin(botHeading);
        double rotatedY = driveX * Math.sin(botHeading) + driveY * Math.cos(botHeading);
        double denominator = Math.max(Math.abs(rotatedY) + Math.abs(rotatedX) + Math.abs(driveTurn), 1)*2;
        powerLF = (rotatedY + rotatedX + driveTurn) / denominator;
        powerLB = (rotatedY - rotatedX + driveTurn) / denominator;
        powerRF = (rotatedY - rotatedX - driveTurn) / denominator;
        powerRB = (rotatedY + rotatedX - driveTurn) / denominator;
        driveLF.setPower(powerLF);
        driveLB.setPower(powerLB);
        driveRF.setPower(powerRF);
        driveRB.setPower(powerRB);

        // in/out
        char intakeColor = 'e';
        
        double outtakePosition[]= {0, 0, 0};
        double intakePosition[] = {0, 0, 0};
        if (gamepad2.right_bumper) mode = "out";
        if (gamepad2.left_bumper) mode = "in";
        if (gamepad2.dpad_up) revolverPosition = 0;
        if (gamepad2.dpad_left) revolverPosition = 1;
        if (gamepad2.dpad_down) revolverPosition = 2;
        if (mode == "in") {
            if (gamepad2.a) {
                if (clawBallColor[revolverPosition] == 'N')
                    clawBallColor[revolverPosition] = intakeColor;
                else
                    clawBallColor[revolverPosition] = 'N';
            }
            revolver.setPosition(intakePosition[revolverPosition]);
        }
        else {
            if (gamepad2.a) {

            }
            revolver.setPosition(outtakePosition[revolverPosition]);
        }
        for (int i = 0; i < 3; i++) {
            if (clawBallColor[i] == 'G') indicatorLights[i].setPosition(0);

        }
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "left front (%.2f) back (%.2f), right front (%.2f) back (%.2f)",
                powerLF, powerLB, powerRF, powerRB);
        telemetry.addData("Heading", botHeading / 3.14159 * 180);
    }

    @Override
    public void stop() {
    }
}
