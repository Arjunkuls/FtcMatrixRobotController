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

@TeleOp(name="Chassis movement", group="Chassis")
public class chassisControl extends LinearOpMode{
    private static double pwr;
    private static DcMotorEx leftFrontMotor;
    private static DcMotorEx leftBackMotor;
    private static DcMotorEx rightFrontMotor;
    private static DcMotorEx rightBackMotor;
    public static DcMotorEx turret;
    public static DcMotorEx arm;
    public static DcMotorEx intake;
    public static double armPower = 0.4;
    public static int HIGHEST_POS = 900;
    public static int MIDDLE_POS = 600;
    public static int LOW_POS = 300;
    public static int PICK_POS = 70;

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        // Todo: Config motors on bot
        leftBackMotor = hardwareMap.get(DcMotorEx.class, "left_back_motor");
        leftFrontMotor = hardwareMap.get(DcMotorEx.class, "left_front_motor");
        rightFrontMotor = hardwareMap.get(DcMotorEx.class, "right_front_motor");
        rightBackMotor = hardwareMap.get(DcMotorEx.class, "right_back_motor");
        turret = hardwareMap.get(DcMotorEx.class, "turret");
        arm = hardwareMap.get(DcMotorEx.class, "arm");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Pause till game start
        waitForStart();
        runtime.reset();
        rightFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBackMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//        armControl.init(turret, arm, intake);
        arm.setDirection(DcMotorSimple.Direction.REVERSE);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setTargetPosition(70);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(0.4);

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            detectUserMovement();
            //            armControl.run();
            checkArmAngle();
            fanMovement();
            turntableHandler();
            armRaise();
            telemetry.addData("log:", gamepad1.left_stick_y);
            telemetry.update();
        }
    }

    // Chassis movement obviously
    static void chassisMovement(double leftFrontMotorPower, double leftBackMotorPower, double rightFrontMotorPower, double rightBackMotorPower){
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

        // Power from right joystick * 0.9
        // This ensures a more manageable power range rather than jerky stuff
        pwr = gamepad1.right_stick_y * 0.9;

        // Move forward
        if(gamepad1.left_stick_y < 0 && gamepad1.left_stick_x == 0 && gamepad1.right_stick_y < 0) {
            leftBackPower = -pwr;
            leftFrontPower = -pwr;
            rightFrontPower = -pwr;
            rightBackPower = -pwr;
        }
        // Move Backward
        else if(gamepad1.left_stick_y > 0 && gamepad1.left_stick_x == 0 && gamepad1.right_stick_y < 0){
            leftBackPower = pwr;
            leftFrontPower = pwr;
            rightFrontPower = pwr;
            rightBackPower = pwr;
        }

        // Glide Left
        else if(gamepad1.left_stick_y == 0 && gamepad1.left_stick_x < 0 && gamepad1.right_stick_y < 0){
            leftBackPower = -pwr;
            leftFrontPower = pwr;
            rightFrontPower = -pwr;
            rightBackPower = pwr;
        }

        // Glide Right
        else if(gamepad1.left_stick_y == 0 && gamepad1.left_stick_x > 0 && gamepad1.right_stick_y < 0){
            leftBackPower = pwr;
            leftFrontPower = -pwr;
            rightFrontPower = pwr;
            rightBackPower = -pwr;
        }

        // Left diagonal
        else if(gamepad1.left_stick_y < 0 && gamepad1.left_stick_x < 0 && gamepad1.right_stick_y < 0){
            leftBackPower = -pwr;
            leftFrontPower = 0;
            rightFrontPower = -pwr;
            rightBackPower = 0;
        }

        // Right diagonal
        else if(gamepad1.left_stick_y < 0 && gamepad1.left_stick_x > 0 && gamepad1.right_stick_y < 0){
            leftBackPower = 0;
            leftFrontPower = -pwr;
            rightFrontPower = 0;
            rightBackPower = -pwr;
        }

        // Right back diagonal
        else if(gamepad1.left_stick_y > 0 && gamepad1.left_stick_x > 0 && gamepad1.right_stick_y < 0){
            leftBackPower = 0;
            leftFrontPower = pwr;
            rightFrontPower = 0;
            rightBackPower = pwr;
        }

        // Left Back diagonal
        else if(gamepad1.left_stick_y > 0 && gamepad1.left_stick_x < 0 && gamepad1.right_stick_y < 0){
            leftBackPower = pwr;
            leftFrontPower = 0;
            rightFrontPower = pwr;
            rightBackPower = 0;
        }

        // Turn Left
        else if(gamepad1.left_bumper && gamepad1.right_stick_y < 0){
            leftBackPower = pwr;
            leftFrontPower = pwr;
            rightFrontPower = -pwr;
            rightBackPower = -pwr;
        }

        // Turn Right
        else if(gamepad1.right_bumper && gamepad1.right_stick_y < 0){
            leftBackPower = -pwr;
            leftFrontPower = -pwr;
            rightFrontPower = pwr;
            rightBackPower = pwr;
        }





        // Finally just move the bot
        chassisMovement(leftFrontPower, leftBackPower, rightFrontPower, rightBackPower); //;(
    }
    public void checkArmAngle(){
        if (arm.getCurrentPosition() >= 420){
            if (gamepad2.left_stick_y < 0){
                armPower = 0;
            }
        }
    }

    // If you cant figure this out how are you accessing the code?
    public void fanMovement(){
        if(gamepad2.b){
            intake.setPower(0.4);
        } else {
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
        // Give it some power so it can actually get a move
    }

}

