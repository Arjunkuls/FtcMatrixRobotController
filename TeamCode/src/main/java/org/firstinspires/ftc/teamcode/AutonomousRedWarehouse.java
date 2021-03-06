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

import static org.firstinspires.ftc.teamcode.Constants.ARM_POSITION_COEFF;
import static org.firstinspires.ftc.teamcode.Constants.ARM_POWER;
import static org.firstinspires.ftc.teamcode.Constants.HIGHEST_POS;
import static org.firstinspires.ftc.teamcode.Constants.LOW_POS;
import static org.firstinspires.ftc.teamcode.Constants.MIDDLE_POS;
import static org.firstinspires.ftc.teamcode.Constants.PICK_POS;
import static org.firstinspires.ftc.teamcode.Constants.TURRET_MOVE;
import static org.firstinspires.ftc.teamcode.Constants.WAREHOUSE_POS;
import static org.firstinspires.ftc.teamcode.Constants.armPidfCoefficients;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Matrix Auto Red Warehouse", group="Linear Opmode")
// @Disabled
public class AutonomousRedWarehouse extends LinearOpMode {

    // Timer
    private final ElapsedTime runtime = new ElapsedTime();

    int POSITION = 0;

    // Arm, Turret and Fan/Carousel
    private DcMotorEx arm, turret, intake;

    // Color Sensor
    Rev2mDistanceSensor distance1;
    Rev2mDistanceSensor distance2;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Chassis
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // Arm and Turret Config
        arm = hardwareMap.get(DcMotorEx.class, "arm");
        arm.setDirection(DcMotorSimple.Direction.REVERSE);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, armPidfCoefficients);
        arm.setPositionPIDFCoefficients(ARM_POSITION_COEFF);

        turret = hardwareMap.get(DcMotorEx.class, "turret");
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intake = hardwareMap.get(DcMotorEx.class, "intake");

        // Get arm and turret ready
        arm.setTargetPosition(PICK_POS);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(ARM_POWER);

        turret.setTargetPosition(0);
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turret.setPower(TURRET_MOVE);

        // Color Config
        distance1 = hardwareMap.get(Rev2mDistanceSensor.class, "distance1");
        distance2 = hardwareMap.get(Rev2mDistanceSensor.class, "distance2");

        // Pose
        Pose2d startPose = new Pose2d(0,0,0);
        drive.setPoseEstimate(startPose);

        // Trajectories
        TrajectorySequence move = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(()->{
                    moveArm(MIDDLE_POS);
                }).waitSeconds(1)
                .lineToConstantHeading(new Vector2d(15, 6))
                .addTemporalMarker(()->{
                    if (getAvgDis(distance1) < 5){
                        moveArm(LOW_POS);
                        POSITION = 0;
                    }
                    else if (getAvgDis(distance2) < 5){
                        moveArm(MIDDLE_POS);
                        POSITION = 1;
                    }
                    else{
                        moveArm(HIGHEST_POS);
                        POSITION = 2;
                    }
                }).waitSeconds(1)
                .lineToConstantHeading(new Vector2d(17.5, 8))
                .build();

        TrajectorySequence topDropper = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(39.5, 20.5, Math.toRadians(-45)))
                .waitSeconds(2)
                .addTemporalMarker(()->{
                    turret.setTargetPosition(WAREHOUSE_POS);
                }).waitSeconds(1)
                .addTemporalMarker(()->{
                    intake.setPower(0.8);
                }).waitSeconds(2)
                .addTemporalMarker(()->{
                    intake.setPower(0);
                })
                .lineToLinearHeading(new Pose2d(14, 8, Math.toRadians(0)))
                .addTemporalMarker(()->{
                    turret.setTargetPosition(0);
                }).waitSeconds(0.5)
                .addTemporalMarker(()->{
                    turret.setTargetPosition(0);
                    arm.setTargetPosition(MIDDLE_POS);
                }).waitSeconds(2)
                .lineToLinearHeading(new Pose2d(-1,0, Math.toRadians(-90)))
                .lineToConstantHeading(new Vector2d(0, -30))
                .lineToConstantHeading(new Vector2d(30, -30))
                .lineToLinearHeading(new Pose2d(30, -50, Math.toRadians(-180)))
                .addTemporalMarker(()->{
                    turret.setTargetPosition(0);
                    arm.setTargetPosition(10);
                })
                .build();
        TrajectorySequence midDropper = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(34.5, 16.5, Math.toRadians(-45)))
                .waitSeconds(2)
                .addTemporalMarker(()->{
                    turret.setTargetPosition(WAREHOUSE_POS);
                }).waitSeconds(1)
                .addTemporalMarker(()->{
                    intake.setPower(0.8);
                }).waitSeconds(2)
                .addTemporalMarker(()->{
                    intake.setPower(0);
                })
                .lineToLinearHeading(new Pose2d(14, 8, Math.toRadians(0)))
                .addTemporalMarker(()->{
                    turret.setTargetPosition(0);
                }).waitSeconds(0.5)
                .addTemporalMarker(()->{
                    turret.setTargetPosition(0);
                    arm.setTargetPosition(MIDDLE_POS);
                }).waitSeconds(2)
                .lineToLinearHeading(new Pose2d(-1,0, Math.toRadians(-90)))
                .lineToConstantHeading(new Vector2d(0, -30))
                .lineToConstantHeading(new Vector2d(30, -30))
                .lineToLinearHeading(new Pose2d(30, -50, Math.toRadians(-180)))
                .addTemporalMarker(()->{
                    turret.setTargetPosition(0);
                    arm.setTargetPosition(10);
                })
                .build();
        TrajectorySequence lowDropper = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(33, 14.5, Math.toRadians(-45)))
                .waitSeconds(2)
                .addTemporalMarker(()->{
                    turret.setTargetPosition(WAREHOUSE_POS);
                }).waitSeconds(1)
                .addTemporalMarker(()->{
                    intake.setPower(0.8);
                }).waitSeconds(2)
                .addTemporalMarker(()->{
                    intake.setPower(0);
                })
                .lineToLinearHeading(new Pose2d(14, 8, Math.toRadians(0)))
                .addTemporalMarker(()->{
                    turret.setTargetPosition(0);
                }).waitSeconds(0.5).addTemporalMarker(()->{
                    arm.setTargetPosition(MIDDLE_POS);
                }).waitSeconds(2)
                .lineToLinearHeading(new Pose2d(-1,0, Math.toRadians(-90)))
                .lineToConstantHeading(new Vector2d(0, -30))
                .lineToConstantHeading(new Vector2d(30, -30))
                .lineToLinearHeading(new Pose2d(30, -50, Math.toRadians(-180)))
                .addTemporalMarker(()->{
                    turret.setTargetPosition(0);
                    arm.setTargetPosition(10);
                })
                .build();
        waitForStart();
        runtime.reset();

        if (isStopRequested()) return;

        drive.followTrajectorySequence(move);
        if (POSITION == 0){
            drive.followTrajectorySequence(lowDropper);
        }
        else if (POSITION == 1){
            drive.followTrajectorySequence(midDropper);
        }
        else if (POSITION == 2){
            drive.followTrajectorySequence(topDropper);
        }
        PoseStorage.currentPose = drive.getPoseEstimate();
    }
    public double getAvgDis(Rev2mDistanceSensor distance){
        double[] distReads = new double[5];
        double sumDist = 0;
        for (int i = 0; i < 5; i++){
            distReads[i] = distance.getDistance(DistanceUnit.INCH);
            sumDist += distReads[i];
        }
        return sumDist/distReads.length;
    }
    public void moveArm(int armPos){
        arm.setTargetPosition(armPos);
        arm.setPower(ARM_POWER);
    }
}