package com._604robotics.robot2018.constants;

import com._604robotics.robotnik.utils.annotations.Unreal;

public class Ports {
    private Ports () {}
    
    // Motors
    // Remember to take note of PDP ports for these too once they are determined
    @Unreal("Wiring yet to be determined. Use Frank wiring for now")
    public static final int DRIVE_FRONT_LEFT_MOTOR = 0;
    @Unreal("Wiring yet to be determined. Use Frank wiring for now")
    public static final int DRIVE_REAR_LEFT_MOTOR = 1;
    @Unreal("Wiring yet to be determined. Use Frank wiring for now")
    public static final int DRIVE_FRONT_RIGHT_MOTOR = 2;
    @Unreal("Wiring yet to be determined. Use Frank wiring for now")
    public static final int DRIVE_REAR_RIGHT_MOTOR = 3;
    @Unreal("Third motor for drive yet to be wired")
    public static final int DRIVE_TOP_LEFT_MOTOR = 4;
    @Unreal("Third motor for drive yet to be wired")
    public static final int DRIVE_TOP_RIGHT_MOTOR = 5;

    // Digital Inputs
    @Unreal("Wiring yet to be determined")
    public static final int ENCODER_LEFT_A = 2;
    @Unreal("Wiring yet to be determined")
    public static final int ENCODER_LEFT_B = 3;
    @Unreal("Wiring yet to be determined")
    public static final int ENCODER_RIGHT_A = 0;
    @Unreal("Wiring yet to be determined")
    public static final int ENCODER_RIGHT_B = 1;
   
    // Analog
    @Deprecated
    @Unreal("Wiring yet to be determined. Use Frank wiring for now"
          + "After development over summer this will probably be gone")
    public static final int HORIZGYRO = 0;
    
    // CAN
    @Unreal("Wiring yet to be determined. Use Frank wiring for now."
          + "These will probably stay the same, but just in case...")
    public static final int COMPRESSOR = 1;
    public static final int PDP_MODULE = 0;
    
    // Elevator
    public static final int ELEVATOR_ENCODER_A = 4;
    public static final int ELEVATOR_ENCODER_B = 5;
    
    public static final int ELEVATOR_MOTOR = 6;
}
