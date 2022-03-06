package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class Constants {
    public static double ARM_POSITION_COEFF = 22;
    public static double ARM_POWER = 0.8;
    public static int HIGHEST_POS = 900;
    public static int MIDDLE_POS = 600;
    public static int LOW_POS = 330;
    public static int PICK_POS = 60;
    public static int DUCK_POS = 1250;

    // Turret power
    public static double TURRET_MOVE = 0.2;
    public static int CAROUSEL_POS = -375;
    public static int WAREHOUSE_POS = 375;

    // Capper
    public static double DROP = 0.25;
    public static double GRIP = 0.98;

    // Arm PID
    public static PIDFCoefficients armPidfCoefficients = new PIDFCoefficients(9.16,0.0828,0.76,0);


}