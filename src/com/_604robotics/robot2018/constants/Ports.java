package com._604robotics.robot2018.constants;

import com._604robotics.robotnik.utils.annotations.Unreal;

public class Ports {
    private Ports () {}
    
    // Motors
    @Unreal("Wiring yet to be determined. Use Frank wiring for now")
    public static final int DRIVE_FRONT_LEFT_MOTOR = 7;
    @Unreal("Wiring yet to be determined. Use Frank wiring for now")
    public static final int DRIVE_REAR_LEFT_MOTOR = 5;
    @Unreal("Wiring yet to be determined. Use Frank wiring for now")
    public static final int DRIVE_FRONT_RIGHT_MOTOR = 0;
    @Unreal("Wiring yet to be determined. Use Frank wiring for now")
    public static final int DRIVE_REAR_RIGHT_MOTOR = 9;

    // Digital Inputs
    @Unreal("Wiring yet to be determined. Use Frank wiring for now")
    public static final int ENCODER_LEFT_A = 2;
    @Unreal("Wiring yet to be determined. Use Frank wiring for now")
    public static final int ENCODER_LEFT_B = 3;
    @Unreal("Wiring yet to be determined. Use Frank wiring for now")
    public static final int ENCODER_RIGHT_A = 0;
    @Unreal("Wiring yet to be determined. Use Frank wiring for now")
    public static final int ENCODER_RIGHT_B = 1;
   
    // Analog
    @Deprecated
    @Unreal("Wiring yet to be determined. Use Frank wiring for now"
          + "After development over summer this will probably be gone")
    public static final int HORIZGYRO = 0;
    
    // CAN
    @Unreal("Wiring yet to be determined. Use Frank wiring for now."
          + "These will probably stay the same, but just in case...")
    public static final int COMPRESSOR = 0;
    public static final int PDP_MODULE = 1;
}