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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;



@TeleOp(name="Chassis movement", group="Chassis")
public class chassisMovement extends LinearOpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontMotor;
    private DcMotor leftBackMotor;
    private DcMotor rightFrontMotor;
    private DcMotor rightBackMotor;


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Todo: Config motors on bot
        leftBackMotor = hardwareMap.get(DcMotor.class, "left_back_motor");
        leftFrontMotor = hardwareMap.get(DcMotor.class, "left_front_motor");
        rightFrontMotor = hardwareMap.get(DcMotor.class, "right_front_motor");
        rightBackMotor = hardwareMap.get(DcMotor.class, "right_back_motor");

        // Reversing right motors
        rightFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBackMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        //Pause till game start
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // Show the elapsed game time.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();

            detectUserMovement();
        }
    }
    // Chassis movement obviously
    void chassisMovement(double leftFrontMotorPower, double leftBackMotorPower, double rightFrontMotorPower, double rightBackMotorPower){
        leftBackMotor.setPower(leftBackMotorPower);
        leftFrontMotor.setPower(leftFrontMotorPower);
        rightFrontMotor.setPower(rightFrontMotorPower);
        rightBackMotor.setPower(rightBackMotorPower);
    }

    // Detect user joystick input and set direction values
    void detectUserMovement(){
        // JAVA makes me ;(
            double leftFrontPower = 0;
            double leftBackPower = 0;
            double rightBackPower = 0;
            double rightFrontPower = 0;

        // Move forward
        if(gamepad1.left_stick_y < 0 && gamepad1.left_stick_x == 0 && gamepad1.right_stick_y < 0) {
            leftBackPower = -gamepad1.right_stick_y;
            leftFrontPower = -gamepad1.right_stick_y;
            rightFrontPower = -gamepad1.right_stick_y;
            rightBackPower = -gamepad1.right_stick_y;
        }
        // Move Backward
        else if(gamepad1.left_stick_y > 0 && gamepad1.left_stick_x == 0 && gamepad1.right_stick_y < 0){
            leftBackPower = -gamepad1.right_stick_y;
            leftFrontPower = -gamepad1.right_stick_y;
            rightFrontPower = -gamepad1.right_stick_y;
            rightBackPower = -gamepad1.right_stick_y;
        }

        // Glide Left
        else if(gamepad1.left_stick_y == 0 && gamepad1.left_stick_x < 0 && gamepad1.right_stick_y < 0){
            leftBackPower = -gamepad1.right_stick_y;
            leftFrontPower = gamepad1.right_stick_y;
            rightFrontPower = -gamepad1.right_stick_y;
            rightBackPower = gamepad1.right_stick_y;
        }

        // Glide Right
        else if(gamepad1.left_stick_y == 0 && gamepad1.left_stick_x > 0 && gamepad1.right_stick_y < 0){
            leftBackPower = gamepad1.right_stick_y;
            leftFrontPower = -gamepad1.right_stick_y;
            rightFrontPower = gamepad1.right_stick_y;
            rightBackPower = -gamepad1.right_stick_y;
        }

        // Left diagonal
        else if(gamepad1.left_stick_y < 0 && gamepad1.left_stick_x < 0 && gamepad1.right_stick_y < 0){
            leftBackPower = -gamepad1.right_stick_y;
            leftFrontPower = 0;
            rightFrontPower = -gamepad1.right_stick_y;
            rightBackPower = 0;
        }

        // Right diagonal
        else if(gamepad1.left_stick_y < 0 && gamepad1.left_stick_x > 0 && gamepad1.right_stick_y < 0){
            leftBackPower = 0;
            leftFrontPower = -gamepad1.right_stick_y;
            rightFrontPower = 0;
            rightBackPower = -gamepad1.right_stick_y;
        }

        // Right back diagonal
        else if(gamepad1.left_stick_y > 0 && gamepad1.left_stick_x > 0 && gamepad1.right_stick_y < 0){
            leftBackPower = 0;
            leftFrontPower = gamepad1.right_stick_y;
            rightFrontPower = 0;
            rightBackPower = gamepad1.right_stick_y;
        }

        // Left Back diagonal
        else if(gamepad1.left_stick_y > 0 && gamepad1.left_stick_x < 0 && gamepad1.right_stick_y < 0){
            leftBackPower = gamepad1.right_stick_y;
            leftFrontPower = 0;
            rightFrontPower = gamepad1.right_stick_y;
            rightBackPower = 0;
        }

        // Turn Left
        else if(gamepad1.dpad_left && gamepad1.right_stick_y < 0){
            leftBackPower = gamepad1.right_stick_y;
            leftFrontPower = gamepad1.right_stick_y;
            rightFrontPower = -gamepad1.right_stick_y;
            rightBackPower = -gamepad1.right_stick_y;
        }

        // Turn Right
        else if(gamepad1.dpad_right && gamepad1.right_stick_y < 0){
            leftBackPower = -gamepad1.right_stick_y;
            leftFrontPower = -gamepad1.right_stick_y;
            rightFrontPower = gamepad1.right_stick_y;
            rightBackPower = gamepad1.right_stick_y;
        }





        // Finally just move the bot
        chassisMovement(leftFrontPower, leftBackPower, rightFrontPower, rightBackPower); //;(
    }
}
