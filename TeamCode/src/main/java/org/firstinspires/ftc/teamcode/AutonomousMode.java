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
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;


@Config
@Autonomous(name="Autonomous", group="Linear Opmode")
public class AutonomousMode extends LinearOpMode {

    public static DcMotorEx turret;
    public static DcMotorEx arm;
    public static DcMotorEx intake;
    public static double armPower = 0.4;
    public static int HIGHEST_POS = 900;
    public static int MIDDLE_POS = 600;
    public static int LOW_POS = 300;
    public static int PICK_POS = 70;
    public static int xCoord1 = 22;
    public static int yCoord1 = -18;
    public static int POSITION = 0;
    public static int SKY_HIGH = 1300;
    public static int CAROUSEL_POS = -420;
    public static double ARM_POWER = 0.8;
    public static double TURRET_MOVE = 0.8;

    Rev2mDistanceSensor distance1;
    Rev2mDistanceSensor distance2;


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

        distance1 = hardwareMap.get(Rev2mDistanceSensor.class, "distance1");
        distance2 = hardwareMap.get(Rev2mDistanceSensor.class, "distance2");

        TrajectorySequence AutoAwesome = Drive.trajectorySequenceBuilder(Start)
                .addTemporalMarker(() -> {
                    arm.setTargetPosition(MIDDLE_POS);
                })
                .waitSeconds(1)
                .lineToConstantHeading(new Vector2d(15.5, -4.5))
                .addTemporalMarker(() -> {
                    if(getAvgDis(distance1) < 3) {
                        arm.setTargetPosition(MIDDLE_POS);
                        POSITION = 1;
                    }
                    else if (getAvgDis(distance2) < 3) {
                        arm.setTargetPosition(HIGHEST_POS);
                        POSITION = 2;
                    } else {
                        arm.setTargetPosition(LOW_POS);
                        POSITION = 0;
                    }
                }).waitSeconds(2)
                .build();
        TrajectorySequence highDropper = Drive.trajectorySequenceBuilder(Drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(27.5, -10.5, Math.toRadians(45)))
                .waitSeconds(2)
                .addTemporalMarker(()->{
                    turret.setTargetPosition(CAROUSEL_POS);
                }).waitSeconds(0.5)
                .addTemporalMarker(()->{
                    intake.setPower(0.8);
                }).waitSeconds(2)
                .addTemporalMarker(()->{
                    intake.setPower(0);
                })
                .lineToLinearHeading(new Pose2d(14, -8, Math.toRadians(0)))
                .addTemporalMarker(()->{
                    turret.setTargetPosition(0);
                }).waitSeconds(0.5)
                .lineToConstantHeading(new Vector2d(5, 24))
                .addTemporalMarker(()->{
                    turret.setTargetPosition(0);
                    turret.setPower(TURRET_MOVE);
                    arm.setTargetPosition(1200);
                    arm.setPower(ARM_POWER);
                }).waitSeconds(4)
                .addTemporalMarker(()->{
                    intake.setPower(1);
                }).waitSeconds(4)
                .addTemporalMarker(()->{
                    intake.setPower(0);
                })
                .lineToLinearHeading(new Pose2d(27, 27, Math.toRadians(-45)))
                .addTemporalMarker(()->{
                    arm.setTargetPosition(10);
                    arm.setPower(ARM_POWER);
                })
                .waitSeconds(5)
                .build();

        TrajectorySequence midDropper = Drive.trajectorySequenceBuilder(Drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(27, -9, Math.toRadians(45)))
                .waitSeconds(2)
                .addTemporalMarker(()->{
                    intake.setPower(0.6);
                }).waitSeconds(2)
                .addTemporalMarker(()->{
                    intake.setPower(0);
                })
                .lineToLinearHeading(new Pose2d(14, -8, Math.toRadians(0)))
                .lineToConstantHeading(new Vector2d(5, 24))
                .addTemporalMarker(()->{
                    turret.setTargetPosition(0);
                    turret.setPower(TURRET_MOVE);
                    arm.setTargetPosition(1200);
                    arm.setPower(ARM_POWER);
                }).waitSeconds(4)
                .addTemporalMarker(()->{
                    intake.setPower(1);
                }).waitSeconds(4)
                .addTemporalMarker(()->{
                    intake.setPower(0);
                })
                .lineToLinearHeading(new Pose2d(27, 27, Math.toRadians(-45)))
                .addTemporalMarker(()->{
                    arm.setTargetPosition(10);
                    arm.setPower(ARM_POWER);
                })
                .waitSeconds(5)
                .build();

        TrajectorySequence lowDropper = Drive.trajectorySequenceBuilder(Drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(26.5, -8, Math.toRadians(45)))
                .waitSeconds(2)
                .addTemporalMarker(()->{
                    intake.setPower(0.6);
                }).waitSeconds(2)
                .addTemporalMarker(()->{
                    intake.setPower(0);
                })
                .lineToLinearHeading(new Pose2d(14, -8, Math.toRadians(0)))
                .lineToConstantHeading(new Vector2d(5, 24))
                .addTemporalMarker(()->{
                    turret.setTargetPosition(0);
                    turret.setPower(TURRET_MOVE);
                    arm.setTargetPosition(1200);
                    arm.setPower(ARM_POWER);
                }).waitSeconds(4)
                .addTemporalMarker(()->{
                    intake.setPower(1);
                }).waitSeconds(4)
                .addTemporalMarker(()->{
                    intake.setPower(0);
                })
                .lineToLinearHeading(new Pose2d(27, 27, Math.toRadians(-45)))
                .addTemporalMarker(()->{
                    arm.setTargetPosition(10);
                    arm.setPower(ARM_POWER);
                })
                .waitSeconds(5)
                .build();



        // Pause till game start
        waitForStart();
        runtime.reset();
        if(isStopRequested())   return;
        Drive.followTrajectorySequence(AutoAwesome);
    }
//    TODO: Promixity Sir
    public double getAvgDis(Rev2mDistanceSensor distance) {
        double [] colorReads = new double[5];
        double sumColor = 0;
        for (int i = 0; i < 5; i++) {
            colorReads[i] = distance.getDistance(DistanceUnit.INCH);
            sumColor += colorReads[i];
        }
        return sumColor / colorReads.length;
    }

}
