package com._604robotics.robot2018.constants;

import com._604robotics.robotnik.utils.AutonMovement;
import com._604robotics.robotnik.utils.annotations.Unreal;

public class Calibration {
    private Calibration () {}

    public static final double TELEOP_DEADBAND = 0.3;
    public static final double TELEOP_FACTOR   = -1;
    
    public static final double DRIVE_MOVE_PID_P = 0.005;
    public static final double DRIVE_MOVE_PID_I = 0;
    public static final double DRIVE_MOVE_PID_D = 0.01;
    public static final double DRIVE_MOVE_PID_MAX = 0.5;
    public static final double DRIVE_MOVE_TOLERANCE = 20;

    // Rotate PID is now calibrated-don't touch
    public static final double DRIVE_ROTATE_PID_P = 0.01;
    public static final double DRIVE_ROTATE_PID_I = 0;
    public static final double DRIVE_ROTATE_PID_D = 0.018;
    public static final double DRIVE_ROTATE_PID_MAX = 0.3;// was 0.5
    public static final double DRIVE_ROTATE_TOLERANCE = 20;

    public static final double DRIVE_PID_AFTER_TIMING = 1.5;
    public static final double DRIVE_PID_SAMPLE_RATE = 0.01;

    public static final double DRIVE_MOVE_STILL_TARGET = 0;
    public static final double DRIVE_ROTATE_STILL_TARGET = 0;
    
    /*
     * 2.5 in diameter of wheels
     * width input = sqrt(25.23^2+(9.06)^2)
     * 25.23 is width, 9.06 is wheel spacing
     */
    @Unreal("Width and wheelRadius need to be adjusted."
            + "Remaining two parameters are to be empirically determined if necessary.")
    public static final AutonMovement.DriveTrainProperties DRIVE_PROPERTIES
    = new AutonMovement.DriveTrainProperties(490, 26.08, 2.5, 20.767, 8.323); // second to last = coefficient second value = offset
    static {
        System.out.println("Clicks over inches is "+DRIVE_PROPERTIES.getClicksOverInches());
        System.out.println("Clicks over degrees is "+DRIVE_PROPERTIES.getDegreesOverClicks());
    }
    
    // Testing targets
    public static final double DRIVE_ROTATE_LEFT_TARGET
    = AutonMovement.degreesToClicks(DRIVE_PROPERTIES, 360);
    public static final double DRIVE_ROTATE_RIGHT_TARGET
    = AutonMovement.degreesToClicks(DRIVE_PROPERTIES, -360);
    public static final double DRIVE_MOVE_FORWARD_TARGET
    = AutonMovement.inchesToClicks(DRIVE_PROPERTIES, 72);
    public static final double DRIVE_MOVE_BACKWARD_TARGET
    = AutonMovement.inchesToClicks(DRIVE_PROPERTIES, -72);
    
    // Empirical auton mode
    public static final double DRIVE_MOVE_FORWARD_TEST_INCHES = AutonMovement.empericalInchesToClicks(DRIVE_PROPERTIES, 36);
    
    // Elevator
    public static final double ELEVATOR_RATE_TARGET = 500;
    public static final double ELEVATOR_RATE_TOLERANCE = 50;
    
    public static final double ELEVATOR_P = 0.0125;
    public static final double ELEVATOR_I = 0.005;
    public static final double ELEVATOR_D = 0.0125;
    public static final double ELEVATOR_MIN_INTEGRAL = -10;
    public static final double ELEVATOR_MAX_INTEGRAL = 10;
}
