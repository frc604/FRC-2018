package com._604robotics.robot2018.constants;

import com._604robotics.robotnik.utils.AutonMovement;
import com._604robotics.robotnik.utils.annotations.Unreal;
import com._604robotics.robotnik.utils.annotations.Untested;

public class Calibration {
    private Calibration () {}

    public static final double TELEOP_DRIVE_DEADBAND = 0.3;
    public static final double TELEOP_MANIP_DEADBAND = 0.11;
    public static final double TELEOP_FACTOR = -1;
    
    public static final double DRIVE_MOVE_PID_P = 0.0045;
    public static final double DRIVE_MOVE_PID_I = 0;
    public static final double DRIVE_MOVE_PID_D = 0.00;
    public static final double DRIVE_MOVE_PID_MAX = 0.7;
    public static final double DRIVE_MOVE_TOLERANCE = 100;

    // Rotate PID is now calibrated-don't touch
    public static final double DRIVE_ROTATE_PID_P = 0.003; // 0.003 / 0.005 / 0.01
    public static final double DRIVE_ROTATE_PID_I = 0;
    public static final double DRIVE_ROTATE_PID_D = 0.01; // 0.005
    public static final double DRIVE_ROTATE_PID_MAX = 0.4;// was 0.5
    public static final double DRIVE_ROTATE_TOLERANCE = 10;

    public static final double DRIVE_PID_AFTER_TIMING = 0.5;
    public static final double DRIVE_PID_SAMPLE_RATE = 0.01;

    public static final double DRIVE_MOVE_STILL_TARGET = 0;
    public static final double DRIVE_ROTATE_STILL_TARGET = 0;
    
    public static final double DRIVE_MOTOR_RAMP = 4;
    
    /*
     * 2.5 in diameter of wheels
     * 25 in width parameter found empirically
     * XXX: Empirical parameters have not been updated AT ALL because they have been unnecessary so far
     */
    public static final AutonMovement.DriveTrainProperties DRIVE_PROPERTIES
    = new AutonMovement.DriveTrainProperties(490, 25.22, 2.5, 20.767, 8.323);//26.05
    // second to last = coefficient, second value = offset
    // Width was 26.7
    static {
        System.out.println("Clicks over inches is "+DRIVE_PROPERTIES.getClicksOverInches());
        System.out.println("Clicks over degrees is "+DRIVE_PROPERTIES.getDegreesOverClicks());
    }
    
    // Testing targets
    // 340 degrees in code is 360 degrees irl at low level of rotation
    public static final double DRIVE_ROTATE_LEFT_TARGET
    = AutonMovement.degreesToClicks(DRIVE_PROPERTIES, -90);
    public static final double DRIVE_ROTATE_RIGHT_TARGET
    = AutonMovement.degreesToClicks(DRIVE_PROPERTIES, 90);
    public static final double DRIVE_MOVE_FORWARD_TARGET
    = AutonMovement.inchesToClicks(DRIVE_PROPERTIES, 72);
    public static final double DRIVE_MOVE_BACKWARD_TARGET
    = AutonMovement.inchesToClicks(DRIVE_PROPERTIES, -72);
    
    // Initial forward direction for switch
    public static final double DRIVE_MOVE_FORWARD_SWITCH_INCHES
     = AutonMovement.inchesToClicks(DRIVE_PROPERTIES, 14*12+1); // 168+1 in
    
    // Elevator
    public static final double ELEVATOR_RATE_TARGET = 500;
    public static final double ELEVATOR_RATE_TOLERANCE = 50;

    // Elevator steady-state power is ~0.1 without arm attached
    // Elevator steady-state power is ~0.2 with 15 lbs attached
    // Multiply above by 0.8 since gear ratio was increased from 16 to 20
    public static final double ELEVATOR_P = 0.00008;
    public static final double ELEVATOR_I = 0.00002;
    public static final double ELEVATOR_D = 0.00000;
    
    // Bound I term motor output to 1
    public static final double ELEVATOR_MAX_SUM = 0.25/ELEVATOR_I;
    // I term which props up elevator should never be negative
    // Needs to be positive as well to counter the weight
    public static final double ELEVATOR_RESET_SUM = 0.2/ELEVATOR_I;
    public static final double ELEVATOR_MIN_SUM = 0.12/ELEVATOR_I;
    public static final double ELEVATOR_PID_PERIOD = 0.02;
    // Lower speed going down due to weight
    public static final double ELEVATOR_MIN_SPEED = -0.1;
    public static final double ELEVATOR_MAX_SPEED = 0.8;
    
    public static final double ELEVATOR_TARGET_SPEED = 0.5;
    public static final int ELEVATOR_CLICK_TOLERANCE = 5;
    
    // Prefer to be at the bottom so push into hard stop
    public static final double ELEVATOR_ENCODER_ZERO = 915;
    public static final double ELEVATOR_LOW_TARGET = 0;
    public static final double ELEVATOR_BUMPER_CLEAR = 3000;
    public static final double ELEVATOR_RAISE_TARGET = 6000;
    public static final double ELEVATOR_SWITCH_CLEAR = 19100;
    public static final double ELEVATOR_MID_TARGET = 14000;
    public static final double ELEVATOR_HIGH_TARGET = 32000;
    //@Unreal("Find more reasonable time or eliminate hold part altogether")
    //public static final double ELEVATOR_PID_CONTINUE = 10;
    public static final double ELEVATOR_PID_TIME_RUN = 5;
    public static final int ELEVATOR_RESET_TOLERANCE = 300;
    
    
    public static final double ARM_P = 0.0001; // 0.00009
    public static final double ARM_I = 0.00004;
    public static final double ARM_D = 0.00002;
    // This is multiplication by a cosine factor
    public static final double ARM_F = 0.25;
    public static final double ARM_ENCODER_FULL_ROT=2*4096*54/30;
    /* Arm */
    // Bound I term motor output to 0.15
    public static final double ARM_MAX_SUM = 0.08/ARM_I;
    public static final double ARM_MIN_SUM = -0.06/ARM_I;
    public static final double ARM_PID_PERIOD = 0.02;
    // Lower speed going down due to weight
    public static final double ARM_MIN_SPEED = -0.07;
    public static final double ARM_MAX_SPEED = 0.8;

    public static final double ARM_RESET_SUM = 0.02/ARM_I;
    public static final double ARM_CLICK_TOLERANCE = 50;
    @Untested("Verify after using SimultaneousCoordinator")
    public static final double ARM_PID_TIME_RUN = 5;
    
    // Low will be negative, high will be positive, zero is horizontal
    // 4096 clicks/rot * 54/30 is 7372.8
    // Assuming 60 degree increments for now
    public static final double ARM_ENCODER_ZERO = -400;
    public static final double ARM_LOW_TARGET = 0;
    public static final double ARM_MID_TARGET = 2300; //0
    public static final double ARM_BALANCE_TARGET = 3000;
    public static final double ARM_CLEAR_TARGET = 500;
    public static final double ARM_HIGH_TARGET = 6300;
    
    /* Intake */
    public static final double INTAKE_PASSIVE_POWER = 0;
    
}
