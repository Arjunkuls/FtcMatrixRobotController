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

package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Config
@Autonomous(name="AutonomousBlue", group="Linear Opmode")
public class AutonomousModeBlue extends LinearOpMode {

    public static DcMotorEx turret;
    public static DcMotorEx arm;
    public static DcMotorEx intake;
    public static double armPower = 0.4;
    public static int HIGHEST_POS = 900;
    public static int MIDDLE_POS = 600;
    public static int LOW_POS = 300;
    public static int PICK_POS = 70;
    public static int xCoord1 = 22;
    public static int yCoord1 = 18;


    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    FtcDashboard boarding = FtcDashboard.getInstance();

    @Override
    public void runOpMode() {
        telemetry = boarding.getTelemetry();

        SampleMecanumDrive Drive = new SampleMecanumDrive(hardwareMap);
        Pose2d Start = new Pose2d(0, 0, Math.toRadians(0));
        Drive.setPoseEstimate(Start);
        turret = hardwareMap.get(DcMotorEx.class, "turret");
        arm = hardwareMap.get(DcMotorEx.class, "arm");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setDirection(DcMotorSimple.Direction.REVERSE);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setTargetPosition(70);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(0.4);

        TrajectorySequence AutoAwesome = Drive.trajectorySequenceBuilder(Start)
                .forward(15.5)
                //TODO: Do barcode stuff
                .lineToLinearHeading(new Pose2d(xCoord1,yCoord1, Math.toRadians(45)))
                //TODO: Dump the object
                .lineToLinearHeading(new Pose2d(0, -20, Math.toRadians(60)))
                //TODO: Moving carousel
                .lineToLinearHeading(new Pose2d(26, -26, Math.toRadians(0)))
                .build();

        // Pause till game start
        waitForStart();
        runtime.reset();
        if(isStopRequested())   return;
        Drive.followTrajectorySequence(AutoAwesome);
    }
}
