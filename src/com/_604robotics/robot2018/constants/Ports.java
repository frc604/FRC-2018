package com._604robotics.robot2018.constants;

import com._604robotics.robotnik.utils.annotations.Unreal;

public class Ports {
    private Ports () {}
    
    // Motors
    // Remember to take note of PDP ports for these too once they are determined
    @Unreal("Wiring yet to be determined. Use practice bot wiring for now")
    public static final int DRIVE_FRONT_LEFT_MOTOR = 0;
    @Unreal("Wiring yet to be determined. Use practice bot wiring for now")
    public static final int DRIVE_REAR_LEFT_MOTOR = 1;
    @Unreal("Wiring yet to be determined. Use practice bot wiring for now")
    public static final int DRIVE_FRONT_RIGHT_MOTOR = 2;
    @Unreal("Wiring yet to be determined. Use practice bot wiring for now")
    public static final int DRIVE_REAR_RIGHT_MOTOR = 3;

    // Digital Inputs
    @Unreal("Wiring yet to be determined")
    public static final int ENCODER_LEFT_A = 2;
    @Unreal("Wiring yet to be determined")
    public static final int ENCODER_LEFT_B = 3;
    @Unreal("Wiring yet to be determined")
    public static final int ENCODER_RIGHT_A = 0;
    @Unreal("Wiring yet to be determined")
    public static final int ENCODER_RIGHT_B = 1;
    
    // CAN
    public static final int COMPRESSOR = 0;
    public static final int PDP_MODULE = 1;
    
    public static final int ELEVATOR_MOTOR_A = 13;
    public static final int ELEVATOR_MOTOR_B = 15;
    //public static final int ELEVATOR_MOTOR_B = 10;
}
