package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class Constants {
    public static double ARM_POSITION_COEFF = 22;
    public static double ARM_POWER = 0.8;
    public static int HIGHEST_POS = 900-70;
    public static int MIDDLE_POS = 600-70;
    public static int LOW_POS = 330-70;
    public static int PICK_POS = 0;
    public static int SKY_HIGH = 1250;

    // Turret power
    public static double TURRET_MOVE = 0.2;
    public static int CAROUSEL_POS = -375;
    public static int WAREHOUSE_POS = 375;

    // Arm PID
    public static PIDFCoefficients armPidfCoefficients = new PIDFCoefficients(9.16,0.0828,0.76,0);


}