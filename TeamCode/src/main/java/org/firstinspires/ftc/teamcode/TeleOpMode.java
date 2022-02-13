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
import com.acmerobotics.roadrunner.drive.Drive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;

@Config
@TeleOp(name="TeleOp", group="Linear Opmode")
public class TeleOpMode extends LinearOpMode {

    public static DcMotorEx turret;
    public static DcMotorEx arm;
    public static DcMotorEx intake;
    public static Servo cap;

// Arm Positions
    public static int HIGHEST_POS = 900;
    public static int MIDDLE_POS = 600;
    public static int LOW_POS = 300;
    public static int PICK_POS = 70;
    public static int SKY_HIGH = 1300;

    // Other variables
    public static double armPower = 1;

    // Pid stuff
    public static PIDFCoefficients armPid = new PIDFCoefficients(5,0,0,0);
    public static double armCoeff = 55;

    public static int xCoord1 = 22;
    public static int yCoord1 = -18;


    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    FtcDashboard boarding = FtcDashboard.getInstance();

    @Override
    public void runOpMode() {
        telemetry = boarding.getTelemetry();

        SampleMecanumDrive Drive = new SampleMecanumDrive(hardwareMap);
        Pose2d Start = new Pose2d(26, 26, Math.toRadians(0));
        Drive.setPoseEstimate(Start);

        turret = hardwareMap.get(DcMotorEx.class, "turret");
        arm = hardwareMap.get(DcMotorEx.class, "arm");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        cap = hardwareMap.get(Servo.class, "cap");
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setDirection(DcMotorSimple.Direction.REVERSE);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, armPid);
        arm.setPositionPIDFCoefficients(armCoeff);
        arm.setTargetPosition(80);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(armPower);

//        TrajectorySequence carouselPos = Drive.trajectorySequenceBuilder(Start)
//                .lineToLinearHeading(new Pose2d(0, 20, Math.toRadians(-60)))
//                .build();

//        TrajectorySequence allianceHub = Drive.trajectorySequenceBuilder(Start)
//                .lineToLinearHeading(new Pose2d(0,-50, Math.toRadians(-90)))
//                .back(20)
//                .lineToLinearHeading(new Pose2d(xCoord1, yCoord1, Math.toRadians(-90)))
//                .build();

//        TrajectorySequence warehouse = Drive.trajectorySequenceBuilder(Start)
//                .lineToLinearHeading(new Pose2d(0,-50, Math.toRadians(-90)))
//                .forward(15)
//                .build();

        while(opModeIsActive()){
            Drive.update();
            // Read pose
            Pose2d poseEstimate = Drive.getPoseEstimate();

            // Create a vector from the gamepad x/y inputs
            // Then, rotate that vector by the inverse of that heading
            Vector2d input = new Vector2d(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x
            ).rotated(-poseEstimate.getHeading());

            // Pass in the rotated input + right stick value for rotation
            // Rotation is not part of the rotated input thus must be passed in separately
            Drive.setWeightedDrivePower(
                    new Pose2d(
                            input.getX(),
                            input.getY(),
                            -gamepad1.right_stick_x
                    )
            );
            soFullOfCap();
            fanMovement();
            turntableHandler();
            armRaise();

            if(gamepad1.y && gamepad1.left_stick_button) {
                TrajectorySequence carouselPos = Drive.trajectorySequenceBuilder(Drive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(0, 20, Math.toRadians(-60)))
                        .build();

                Drive.followTrajectorySequence(carouselPos);
            }

            if(gamepad1.x && gamepad1.left_stick_button) {
                TrajectorySequence allianceHub = Drive.trajectorySequenceBuilder(Drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(0,-50, Math.toRadians(-90)))
                .back(20)
                .lineToLinearHeading(new Pose2d(xCoord1, yCoord1, Math.toRadians(-90)))
                .build();
                Drive.followTrajectorySequence(allianceHub);
            }

            if(gamepad1.a && gamepad1.left_stick_button) {
                TrajectorySequence warehouse = Drive.trajectorySequenceBuilder(Drive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(0,-50, Math.toRadians(-90)))
                        .forward(15)
                        .build();
                Drive.followTrajectorySequence(warehouse);
            }
        }

        // Pause till game start
        waitForStart();
        runtime.reset();
        if(isStopRequested())   return;
    }

    public void soFullOfCap(){
        if(gamepad1.dpad_left){
            cap.setPosition(0.93);
        }else if (gamepad1.dpad_right){
            cap.setPosition(0.22);
        }
    }

    // If you cant figure this out how are you accessing the code?
    public void fanMovement(){
        if(gamepad1.left_bumper){
            intake.setPower(-1);
        } else if (gamepad1.right_bumper) {
            intake.setPower(1);
        }else{
            intake.setPower(0);
        }

    }

    public void turntableHandler(){
        if(gamepad1.left_trigger != 0){
            turret.setPower(gamepad1.left_trigger);
        }
        else if(gamepad1.right_trigger != 0){
            turret.setPower(gamepad1.right_trigger);
        }
        else{
            turret.setPower(0);
        }
    }

    public void armRaise(){
        //Highest Position
        if (gamepad1.y && !gamepad1.left_stick_button) {
            arm.setTargetPosition(HIGHEST_POS);
        }
        //Middle Position
        else if (gamepad1.x && !gamepad1.left_stick_button) {
            arm.setTargetPosition(MIDDLE_POS);
        }
        // Lowest Position
        else if (gamepad1.a && !gamepad1.left_stick_button) {
            arm.setTargetPosition(LOW_POS);
        }
        // Scoop Position
        else if (gamepad1.b && !gamepad1.left_stick_button) {
            arm.setTargetPosition(PICK_POS);
        }
        else if (gamepad1.start){
            arm.setTargetPosition(SKY_HIGH);
        } else if (gamepad1.dpad_up) {
            arm.setTargetPosition(arm.getCurrentPosition() + 20);
        } else if (gamepad1.dpad_down) {
            arm.setTargetPosition(arm.getCurrentPosition() - 20);
        }
        // Give it some power so it can actually get a move on
    }



}
