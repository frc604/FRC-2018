package com._604robotics.robot2018.constants;

import com._604robotics.robotnik.utils.annotations.Unreal;

public class Ports {
    private Ports () {}
    
    // Solenoids
    public static final int SHIFTER_A = 0;
    public static final int SHIFTER_B = 2;
    
    public static final int CLAMP_A = 1;
    public static final int CLAMP_B = 3;
    
    // Motors
    // Remember to take note of PDP ports for these too once they are determined
    public static final int DRIVE_FRONT_LEFT_MOTOR = 2;
    public static final int DRIVE_REAR_LEFT_MOTOR = 3;
    public static final int DRIVE_FRONT_RIGHT_MOTOR = 0;
    public static final int DRIVE_REAR_RIGHT_MOTOR = 1;

    // Digital Inputs
    public static final int ENCODER_LEFT_A = 0;
    public static final int ENCODER_LEFT_B = 1;
    public static final int ENCODER_RIGHT_A = 2;
    public static final int ENCODER_RIGHT_B = 3;
    
    // CAN
    public static final int COMPRESSOR = 50;
    public static final int PDP_MODULE = 51;
    
    public static final int ELEVATOR_MOTOR_A = 13;
    public static final int ELEVATOR_MOTOR_B = 15;
    
    public static final int INTAKE_MOTOR_A = 12;
    public static final int INTAKE_MOTOR_B = 14;
    
    public static final int ARM_MOTOR_A = 11;
    public static final int ARM_MOTOR_B = 10;
}
