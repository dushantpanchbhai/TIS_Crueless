package org.firstinspires.ftc.teamcode.TestCodes.Mechanisms;

import com.acmerobotics.dashboard.config.Config;

import org.checkerframework.checker.index.qual.PolyUpperBound;

@Config
public class ArmConstants {

//    ##### EXTENSION + DROPPER CONSTANTS ############
    public static int extension_max_pos = -1029;
    public static int extension_low_pos = 50;

    public static double servo_dropper_pickup_pos = 0.44;
    public static double servo_dropper_mid = 0.6288;
    public static double servo_dropper_drop_pos = 1;


    public static double servo_arm_pickup_pos = 0.26;
    public static double servo_arm_mid = 0.4366;
    public static double servo_arm_drop_pos_wait = 1;
    public static double servo_arm_drop_pos_drop = 0;

//    ##### TURN TABLE CONSTANTS ############
    public static int turn_table_inc = 40;
}
