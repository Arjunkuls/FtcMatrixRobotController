package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(name="Chassis movement", group="Chassis")
public class chassisControl extends LinearOpMode{
    private static double pwr;
//    private static DcMotorEx leftFrontMotor;
//    private static DcMotorEx leftBackMotor;
//    private static DcMotorEx rightFrontMotor;
//    private static DcMotorEx rightBackMotor;
    public static PIDFCoefficients armPid = new PIDFCoefficients(5,0,0,0);
    public static double armCoeff = 55;
    public static double armPower = 0.7;
    public static DcMotorEx turret;
    public static DcMotorEx arm;
    public static DcMotorEx intake;
    public static Servo cap;
    public static int HIGHEST_POS = 900;
    public static int MIDDLE_POS = 600;
    public static int LOW_POS = 300;
    public static int PICK_POS = 50;
    public static int SKY_HIGH = 1300;

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        SampleMecanumDrive Drive = new SampleMecanumDrive(hardwareMap);
        Pose2d Start = new Pose2d(0, 0, Math.toRadians(0));
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

        // Pause till game start
        waitForStart();
        runtime.reset();



        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
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
            Drive.update();
//            detectUserMovement();
            //            armControl.run();
            checkArmAngle();
            fanMovement();
            turntableHandler();
            armRaise();
            soFullOfCap();
            telemetry.addData("log:", gamepad1.left_stick_y);
            telemetry.update();
        }
    }

    // Chassis movement obviously
//    static void chassisMovement(double leftFrontMotorPower, double leftBackMotorPower, double rightFrontMotorPower, double rightBackMotorPower){
//        leftBackMotor.setPower(leftBackMotorPower);
//        leftFrontMotor.setPower(leftFrontMotorPower);
//        rightFrontMotor.setPower(rightFrontMotorPower);
//        rightBackMotor.setPower(rightBackMotorPower);
//    }

    // Detect user joystick input and set direction values
//    void detectUserMovement(){
//        // JAVA makes me ;(
//        double leftFrontPower = 0;
//        double leftBackPower = 0;
//        double rightBackPower = 0;
//        double rightFrontPower = 0;
//
//        // Power from right joystick * 0.9
//        // This ensures a more manageable power range rather than jerky stuff
//        pwr = gamepad1.right_stick_y * 0.9;
//
//        // Move forward
//        if(gamepad1.left_stick_y < 0 && gamepad1.left_stick_x == 0 && gamepad1.right_stick_y < 0) {
//            leftBackPower = -pwr;
//            leftFrontPower = -pwr;
//            rightFrontPower = -pwr;
//            rightBackPower = -pwr;
//        }
//        // Move Backward
//        else if(gamepad1.left_stick_y > 0 && gamepad1.left_stick_x == 0 && gamepad1.right_stick_y < 0){
//            leftBackPower = pwr;
//            leftFrontPower = pwr;
//            rightFrontPower = pwr;
//            rightBackPower = pwr;
//        }
//
//        // Glide Left
//        else if(gamepad1.left_stick_y == 0 && gamepad1.left_stick_x < 0 && gamepad1.right_stick_y < 0){
//            leftBackPower = -pwr;
//            leftFrontPower = pwr;
//            rightFrontPower = -pwr;
//            rightBackPower = pwr;
//        }
//
//        // Glide Right
//        else if(gamepad1.left_stick_y == 0 && gamepad1.left_stick_x > 0 && gamepad1.right_stick_y < 0){
//            leftBackPower = pwr;
//            leftFrontPower = -pwr;
//            rightFrontPower = pwr;
//            rightBackPower = -pwr;
//        }
//
//        // Left diagonal
//        else if(gamepad1.left_stick_y < 0 && gamepad1.left_stick_x < 0 && gamepad1.right_stick_y < 0){
//            leftBackPower = -pwr;
//            leftFrontPower = 0;
//            rightFrontPower = -pwr;
//            rightBackPower = 0;
//        }
//
//        // Right diagonal
//        else if(gamepad1.left_stick_y < 0 && gamepad1.left_stick_x > 0 && gamepad1.right_stick_y < 0){
//            leftBackPower = 0;
//            leftFrontPower = -pwr;
//            rightFrontPower = 0;
//            rightBackPower = -pwr;
//        }
//
//        // Right back diagonal
//        else if(gamepad1.left_stick_y > 0 && gamepad1.left_stick_x > 0 && gamepad1.right_stick_y < 0){
//            leftBackPower = 0;
//            leftFrontPower = pwr;
//            rightFrontPower = 0;
//            rightBackPower = pwr;
//        }
//
//        // Left Back diagonal
//        else if(gamepad1.left_stick_y > 0 && gamepad1.left_stick_x < 0 && gamepad1.right_stick_y < 0){
//            leftBackPower = pwr;
//            leftFrontPower = 0;
//            rightFrontPower = pwr;
//            rightBackPower = 0;
//        }
//
//        // Turn Left
//        else if(gamepad1.left_bumper && gamepad1.right_stick_y < 0){
//            leftBackPower = pwr;
//            leftFrontPower = pwr;
//            rightFrontPower = -pwr;
//            rightBackPower = -pwr;
//        }
//
//        // Turn Right
//        else if(gamepad1.right_bumper && gamepad1.right_stick_y < 0){
//            leftBackPower = -pwr;
//            leftFrontPower = -pwr;
//            rightFrontPower = pwr;
//            rightBackPower = pwr;
//        }
//
//
//
//
//
//        // Finally just move the bot
//        chassisMovement(leftFrontPower, leftBackPower, rightFrontPower, rightBackPower); //;(
//    }
    public void checkArmAngle(){
        if (arm.getCurrentPosition() >= 420){
            if (gamepad2.left_stick_y < 0){
                armPower = 0;
            }
        }
    }

    public void soFullOfCap(){
        if(gamepad2.dpad_up){
            cap.setPosition(0.93);
        }else if (gamepad2.dpad_down){
            cap.setPosition(0.22);
        }
    }

    // If you cant figure this out how are you accessing the code?
    public void fanMovement(){
        if(gamepad2.b){
            intake.setPower(1);
        } else if (gamepad2.left_trigger > 0.1) {
            intake.setPower(-1);
        }else{
            intake.setPower(0);
        }
    }

    //You're actually stupid
    public void turntableHandler(){
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
    public void armRaise(){
        //Highest Position
        if (gamepad2.y) {
            arm.setTargetPosition(HIGHEST_POS);
        }
        //Middle Position
        else if (gamepad2.x) {
            arm.setTargetPosition(MIDDLE_POS);
        }
        // Lowest Position
        else if (gamepad2.a) {
            arm.setTargetPosition(LOW_POS);
        }
        // Scoop Position
        else if (gamepad2.right_trigger > 0) {
            arm.setTargetPosition(PICK_POS);
        }
        else if (gamepad1.x){
            arm.setTargetPosition(SKY_HIGH);
        } else if (gamepad2.dpad_right) {
            arm.setTargetPosition(arm.getCurrentPosition() - 10);
        } else if (gamepad2.dpad_left) {
            arm.setTargetPosition(arm.getCurrentPosition() + 10);
        }
        // Give it some power so it can actually get a move
    }

}

