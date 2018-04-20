package com._604robotics.robot2018.constants;

import com._604robotics.robotnik.utils.AutonMovement;
import com._604robotics.robotnik.utils.annotations.Unreal;
import com._604robotics.robotnik.utils.annotations.Untested;

public class Calibration {
    private Calibration () {}
    
    public static final boolean TANDEM_ACTIVE = true;
    public static final boolean ELEVATOR_HOLD_ACTIVE = true;
    public static final boolean ARM_HOLD_ACTIVE = true;
    
    public static final double TELEOP_DRIVE_DEADBAND = 0.3;
    public static final double TELEOP_MANIP_DEADBAND = 0.2;
    public static final double TELEOP_FACTOR = -1;
    
    public static final double DRIVE_MOVE_PID_P = 0.0045;
    public static final double DRIVE_MOVE_PID_I = 0;
    public static final double DRIVE_MOVE_PID_D = 0.00;
    public static final double DRIVE_MOVE_PID_MAX = 0.85; //0.7
    public static final double DRIVE_MOVE_TOLERANCE = 100;

    // Rotate PID is now calibrated-don't touch
    public static final double DRIVE_ROTATE_PID_P = 0.003; // 0.003 / 0.005 / 0.01
    public static final double DRIVE_ROTATE_PID_I = 0;
    public static final double DRIVE_ROTATE_PID_D = 0.01; // 0.005
    public static final double DRIVE_ROTATE_PID_MAX = 0.4;// was 0.5
    public static final double DRIVE_ROTATE_TOLERANCE = 80;

    public static final double DRIVE_PID_AFTER_TIMING = 0.75;
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
    = new AutonMovement.DriveTrainProperties(490, 26.64, 2.5, 20.767, 8.323);//26.05
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
    = AutonMovement.inchesToClicks(DRIVE_PROPERTIES, 144);
    public static final double DRIVE_MOVE_BACKWARD_TARGET
    = AutonMovement.inchesToClicks(DRIVE_PROPERTIES, -144);
    
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
    public static final double ELEVATOR_PID_PERIOD = 0.02;
    
    // Bound I term motor output
    // I term which props up elevator should never be negative
    // Needs to be positive as well to counter the weight
    public static final double ELEVATOR_MIN_SUM = 0.12/ELEVATOR_I;
    public static final double ELEVATOR_MAX_SUM = 0.25/ELEVATOR_I;
    public static final double ELEVATOR_RESET_SUM = 0.2/ELEVATOR_I;
    // Lower speed going down due to weight
    public static final double ELEVATOR_MIN_SPEED = -0.1;
    public static final double ELEVATOR_MAX_SPEED = 0.8;
    
    // Tolerance for PID controller
    public static final int ELEVATOR_CLICK_TOLERANCE = 50;
    // Reset elevator encoder if it goes below this value
    public static final int ELEVATOR_RESET_TOLERANCE = 200;
    // Prefer to be at the bottom so push into hard stop
    @Unreal("Now using zero function so could have drifted arbitrarily") @Deprecated
    public static final double ELEVATOR_ENCODER_ZERO = 720-70;
    public static final double ELEVATOR_LOW_TARGET = 0;
    public static final double ELEVATOR_BUMPER_CLEAR = 3000;
    public static final double ELEVATOR_RAISE_TARGET = 5000;
    public static final double ELEVATOR_SWITCH_CLEAR = 10000;
    public static final double ELEVATOR_MID_TARGET = 14000;
    public static final double ELEVATOR_HIGH_TARGET = 32000;
    
    /* Arm */
    public static final double ARM_P = 0.00014; // 0.0001
    public static final double ARM_I = 0.00004;
    public static final double ARM_D = 0.000022;
    // This is multiplication by a cosine factor
    public static final double ARM_F = 0.25;
    public static final double ARM_TELEOP_OFFSET_ENCODERFAIL = 0.15;
    public static final double ARM_PID_PERIOD = 0.02;
    
    @Unreal("Now using zero function so could have drifted arbitrarily") @Deprecated
    public static final double ARM_ENCODER_ZERO = 3030-2900;
    public static final double ARM_ENCODER_FULL_ROT=2*4096*54/30;
    
    // Lower speed going down due to weight
    public static final double ARM_MIN_SPEED = -0.15;
    public static final double ARM_MAX_SPEED = 0.8;
    // Bound I term motor output
    public static final double ARM_MIN_SUM = -0.06/ARM_I;
    public static final double ARM_MAX_SUM = 0.08/ARM_I;
    public static final double ARM_RESET_SUM = 0.02/ARM_I;
    
    public static final double ARM_CLICK_TOLERANCE = 50;
    
    // Low will be negative, high will be positive, zero is horizontal
    // 4096 clicks/rot * 54/30 * 2 = 14745.6
    // Arm low target should push into the foam
    public static final double ARM_LOW_TARGET = -2400;
    public static final double ARM_BOTTOM_LOCATION = -2170;
    public static final double ARM_MID_TARGET = 200; //0
    @Unreal("Remnants from using arm to facilitate turning") @Deprecated
    public static final double ARM_BALANCE_TARGET = 900;
    public static final double ARM_HIGH_TARGET = 4500;
    public static final double ARM_RAISE_TARGET = 0;
    
    /* Intake */
    public static final double INTAKE_INTAKE_MODIFIER = 0.5;
    public static final double INTAKE_OUTAKE_DRIVER_MODIFIER = 0.25;//0.35
    public static final double INTAKE_OUTAKE_MANIPULATOR_SCALE_MODIFIER = 0.25;
    public static final double INTAKE_OUTAKE_MANIPULATOR_SWITCH_MODIFIER = 0.4;
    public static final double INTAKE_OUTAKE_DRIVER_OVERDRIVE_MODIFIER = 0.8; //0.4
    public static final double INTAKE_OUTAKE_MANIPULATOR_OVERDRIVE_MODIFIER = 0.6;
    public static final double INTAKE_PASSIVE_POWER = 0.1;
    
    /* Marionette */
    public static final boolean AUTO_APPEND_TIMESTAMP = true;
    public static final String SWITCH_LEFT_FILENAME = "switchLeft.marionette";
    public static final String SWITCH_RIGHT_FILENAME = "switchRight.marionette";
    public static final String SCALE_LEFT_FILENAME = "scaleLeft.marionette";
    public static final String SCALE_RIGHT_FILENAME = "scaleRight.marionette";
    
    public static final double PLAYBACK_DELAY = 0.0215;
    public static final int RECORD_DELAY = 25;
    public static final int PLAYBACK_MAXFRAMES = 1200;
    
}
