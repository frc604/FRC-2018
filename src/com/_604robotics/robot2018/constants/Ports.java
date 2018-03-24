package com._604robotics.robot2018.constants;

public class Ports {
    private Ports () {}
    
    /* Solenoids */
    public static final int SHIFTER_A = 6;
    public static final int SHIFTER_B = 7;
    
    public static final int CLAMP_A = 4;
    public static final int CLAMP_B = 5;
    
    /* Victor Motors */
    // Remember to take note of PDP ports for these too once they are determined
    // Drive
    public static final int DRIVE_FRONT_LEFT_MOTOR = 1;
    public static final int DRIVE_REAR_LEFT_MOTOR = 0;
    public static final int DRIVE_FRONT_RIGHT_MOTOR = 3;
    public static final int DRIVE_REAR_RIGHT_MOTOR = 2;
    
    // Intake
    public static final int INTAKE_OUTER_MOTOR_A = 5;
    public static final int INTAKE_OUTER_MOTOR_B = 6;

    /* Digital Inputs */
    public static final int ENCODER_LEFT_A = 0;
    public static final int ENCODER_LEFT_B = 1;
    public static final int ENCODER_RIGHT_A = 2;
    public static final int ENCODER_RIGHT_B = 3;
    
    public static final int ARM_BOTTOM_SWITCH = 4;
    
    /* CAN Motors */
    public static final int COMPRESSOR = 0;
    public static final int PDP_MODULE = 51;
    
    public static final int ELEVATOR_MOTOR_A = 10;
    public static final int ELEVATOR_MOTOR_B = 15;
    
    public static final int INTAKE_INNER_MOTOR_A = 12;
    public static final int INTAKE_INNER_MOTOR_B = 14;
    
    public static final int ARM_MOTOR_A = 13;
    public static final int ARM_MOTOR_B = 11;
}
