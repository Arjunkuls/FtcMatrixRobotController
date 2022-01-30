package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class armControl {

    public static DcMotorEx turret;
    public static DcMotorEx arm;
    public static DcMotorEx intake;
    public static double armPower = 0.4;
    // Init function
    // Run once
    public static void init(DcMotorEx turretTemp, DcMotorEx armTemp, DcMotorEx intakeTemp){
        turret = turretTemp;
        arm = armTemp;
        intake = intakeTemp;

        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setTargetPosition(0);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(armPower);
    }

    // Game Loop
    // Run constantly
    public static void run(){
    checkArmAngle();
    fanMovement();
    turntableHandler();
    armRaise();
    }

    // Ensures arm doesnt raise too high and cause issues in life
    public static void checkArmAngle(){
        if (arm.getCurrentPosition() >= 420){
            if (gamepad2.left_stick_y < 0){
                armPower = 0;
            }
        }
    }

    // If you cant figure this out how are you accessing the code?
    public static void fanMovement(){
        if(gamepad2.b){
            intake.setPower(0.4);
        } else {
            intake.setPower(0);
        }
    }

    //You're actually stupid
    public static void turntableHandler(){
        if(gamepad2.left_bumper){
            turret.setPower(0.4);
        }
        else if(gamepad2.right_bumper){
            turret.setPower(-0.4);
        }
        else{
            turret.setPower(0);
        }
    }

    //Idioto
    public static void armRaise(){
        if(gamepad2.left_stick_y > 0){
            armPower = gamepad1.left_stick_y;
        }
        arm.setPower(armPower);
    }
}
