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
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.util.BNO055IMUUtil;

@Config
@TeleOp(name="TeleOp", group="Linear Opmode")
public class TeleOpMode extends LinearOpMode {

    // Arm Motors
    public static DcMotorEx turret;
    public static DcMotorEx arm;
    public static DcMotorEx intake;


    // Arm Variables
    public static int HIGHEST_POS = Constants.HIGHEST_POS;
    public static int MIDDLE_POS = Constants.MIDDLE_POS;
    public static int LOW_POS = Constants.LOW_POS;
    public static int PICK_POS = Constants.PICK_POS;
    public static int SKY_HIGH = Constants.SKY_HIGH;
    public static double armPower = 1;
    public static int PerpendicularToTheFrickingBot = -420;

    // Variables for P.I.D.
    public static PIDFCoefficients armPid = new PIDFCoefficients(5,0,0,0);
    public static double armCoeff = 55;

    // FTC Essential Variables
    private ElapsedTime runtime = new ElapsedTime();
    FtcDashboard boarding = FtcDashboard.getInstance();

    @Override
    public void runOpMode() {
        telemetry = boarding.getTelemetry();

        // Mecanum Drive Init
        SampleMecanumDrive Drive = new SampleMecanumDrive(hardwareMap);
        Drive.setPoseEstimate(PoseStorage.currentPose);

        // Hardware Mapping
        turret = hardwareMap.get(DcMotorEx.class, "turret");
        arm = hardwareMap.get(DcMotorEx.class, "arm");
        intake = hardwareMap.get(DcMotorEx.class, "intake");

        // Arm init stuff
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setDirection(DcMotorSimple.Direction.REVERSE);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, armPid);
        arm.setPositionPIDFCoefficients(armCoeff);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(armPower);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turret.setTargetPosition(0);
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turret.setPower(0.8);

        // Wait till the program is started
        waitForStart();
        runtime.reset();

        // End code if stopped
        if(isStopRequested())   return;

        // While the program is running
        while(opModeIsActive()){
        Drive.update();

            // Read pose
            Pose2d poseEstimate = Drive.getPoseEstimate();
            double x;
            double y;
            double rx;

            if (Math.abs(gamepad2.left_stick_y) > 0.2) {
                y = -gamepad2.left_stick_y * 0.8;
            } else {
                y = 0;
            }

            if (Math.abs(gamepad2.left_stick_x) > 0.2) {
                x = gamepad2.left_stick_x * 0.8;
            } else {
                x = 0;
            }

            if (Math.abs(gamepad2.right_stick_x) > 0.2) {
                rx = gamepad2.right_stick_x * gamepad2.right_stick_x * gamepad2.right_stick_x;
            } else {
                rx = 0;
            }


            // Create a vector from the gamepad x/y inputs
            // Then, rotate that vector by the inverse of that heading
            Vector2d input = new Vector2d(
                    y,
                    -x
            ).rotated(-poseEstimate.getHeading());

            // Pass in the rotated input + right stick value for rotation
            // Rotation is not part of the rotated input thus must be passed in separately
            Drive.setWeightedDrivePower(
                    new Pose2d(
                            input.getX(),
                            input.getY(),
                            -rx
                    )
            );

            // Call Running Functions
            fanMovement();
            turntableHandler();
            armRaise();

        }

    }



    // Controls the intake, outtake and the carousel
    public void fanMovement(){
        if(gamepad1.left_bumper){
            intake.setPower(-1);
        } else if (gamepad1.right_bumper) {
            intake.setPower(1);
        }
        else if(gamepad2.left_bumper){
            intake.setPower(-1);
        } else if (gamepad2.right_bumper) {
            intake.setPower(1);
        }else{
            intake.setPower(0);
        }
    }

    // It moves the turntable
    public void turntableHandler(){
        if(gamepad1.left_trigger >= 0.8){
            turret.setTargetPosition(turret.getCurrentPosition() + 50);
        }
        else if(gamepad1.right_trigger >= 0.8){
            turret.setTargetPosition(turret.getCurrentPosition() - 50);
        }
        else if (gamepad1.left_stick_button){
            turret.setTargetPosition(-PerpendicularToTheFrickingBot);
        }
        else if (gamepad1.right_stick_button){
            turret.setTargetPosition(PerpendicularToTheFrickingBot);
        }
    }

    public void armRaise(){
        //Highest Position
        if (gamepad1.y) {
            arm.setTargetPosition(HIGHEST_POS);
        }
        //Middle Position
        else if (gamepad1.x) {
            arm.setTargetPosition(MIDDLE_POS);
        }
        // Lowest Position
        else if (gamepad1.a) {
            arm.setTargetPosition(LOW_POS);
        }
        // Scoop Position
        else if (gamepad1.b) {
            arm.setTargetPosition(PICK_POS);
        }
        else if (gamepad1.start){
            arm.setTargetPosition(SKY_HIGH);
        }
        else if (gamepad1.back){
            turret.setTargetPosition(0);
        }

        else if (gamepad1.dpad_up) {
            arm.setTargetPosition(arm.getCurrentPosition() + 20);
        } else if (gamepad1.dpad_down) {
            arm.setTargetPosition(arm.getCurrentPosition() - 20);
        }
        if (gamepad2.start){
            arm.setTargetPosition(SKY_HIGH);
        }
        // Give it some power so it can actually get a move on
    }

    public double getAvgDis(RevColorSensorV3 color) {
        double [] colorReads = new double[5];
        double sumColor = 0;
        for (int i = 0; i < 5; i++) {
            colorReads[i] = color.getDistance(DistanceUnit.INCH);
            sumColor += colorReads[i];
        }
        return sumColor / colorReads.length;
    }



}