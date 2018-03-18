package com._604robotics.robot2018.modules;

import com._604robotics.robotnik.Input;
import com._604robotics.robotnik.Output;
import com._604robotics.robotnik.prefabs.modules.DashboardModule;

public class Dashboard extends DashboardModule {
    public final Input<Integer> leftDriveClicks = addDashboardInput("leftDriveClicks", 0);
    public final Input<Integer> rightDriveClicks = addDashboardInput("rightDriveClicks", 0);

    public final Input<Double> leftDriveRate = addDashboardInput("leftDriveRate", 0.0);
    public final Input<Double> rightDriveRate = addDashboardInput("rightDriveRate", 0.0);

    //public final Input<Double> totalCurrent = addDashboardInput("Current Drawn",0.0);

    public final Input<Boolean> XboxFlipped = addDashboardInput("XboxFlipped", false);
    
    //public final Input<Double> elevatorOffset = addDashboardInput("Elevator Offset", 0.0);
    //public final Input<Double> elevatorUpwardsRange = addDashboardInput("Upwards Range", 0.0);
    //public final Input<Double> elevatorDownwardsRange = addDashboardInput("Downwards Range", 0.0);
    //public final Input<Boolean> elevatorFailsafed = addDashboardInput("Failsafed", false);
    
    public final Input<Double> elevatorRate = addDashboardInput("Elevator Rate", 0.0);
    public final Input<Double> elevatorClicks = addDashboardInput("Elevator Clicks", 0.0);
    
    //public final Input<Boolean> holding = addDashboardInput("Holding", true);
    //public final Input<Double> power = addDashboardInput("Elevator Power", 0.0);
    
    public final Input<Double> armRate = addDashboardInput("Arm Rate", 0.0);
    public final Input<Double> armClicks = addDashboardInput("Arm Clicks", 0.0);
    
    public final Input<String> isClamped = addDashboardInput("Is Clamped", "CLAMPED");
    public final Input<Boolean> clampLightA = addDashboardInput("Clamp Light A", false);
    public final Input<Boolean> clampLightB = addDashboardInput("Clamp Light B", false);
    
    public final Input<Boolean> armEncoderStatus = addDashboardInput("Arm Encoder Status", false);
    public final Input<Boolean> limitPressed = addDashboardInput("LimitPressed", false);
    
    public enum AutonMode {
        OFF,
        // Following are actual strategy selections
        CENTER_SWITCH,
        BACKWARD_CENTER_SWITCH,
        LEFT_SWITCH,
        RIGHT_SWITCH,
        LEFT_SCALE,
        LEFT_SCALE_SAME_ONLY,
        RIGHT_SCALE,
        RIGHT_SCALE_SAME_ONLY,
        // Calibration autons to verify angles and distances
        ROTATE_LEFT_TEST,
        ROTATE_RIGHT_TEST,
        FAILSAFE_FORWARD_12,
        FAILSAFE_BACKWARD_12,
        // Demo auton into which arbitrary stuff can be stashed for testing
        DEMO_NEW_AUTON,
        // Remnants of various tests
        FORWARD_SWITCH,
        // CENTER_SWITCH_LEFT,
        // CENTER_SWITCH_RIGHT,
//        SWITCH_FORWARD,
        SCALE_BACKWARD,
        SCALE_BACKWARD_2,
        NEW_SCALE_BACKWARD,
        SCALE_OPPOSITE,
//        SWERVE_SCALE_OPPOSITE_LEFT,
//        BALANCED_LEFT_TURN_TEST,
//        SWEPT_LEFT_TURN_TEST,
//        BALANCED_SWEPT_LEFT_TURN_TEST,
//        BALANCED_RIGHT_TURN_TEST,
//        SWEPT_RIGHT_TURN_TEST,
//        BALANCED_SWEPT_RIGHT_TURN_TEST,
    }
    
    public enum DriveMode {
        OFF,
        ARCADE,
        TANK,
        DYNAMIC
    }

    public final Output<AutonMode> autonMode = addDashboardOutput("autonMode", AutonMode.OFF, AutonMode.class);
    
    public final Output<DriveMode> driveMode = addDashboardOutput("driveMode", DriveMode.DYNAMIC, DriveMode.class);

    public Dashboard () {
        super(Dashboard.class);
    }
}