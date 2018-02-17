package com._604robotics.robot2018.modules;

import com._604robotics.robotnik.Input;
import com._604robotics.robotnik.Output;
import com._604robotics.robotnik.prefabs.modules.DashboardModule;

public class Dashboard extends DashboardModule {
    public final Input<Integer> leftDriveClicks = addDashboardInput("leftDriveClicks", 0);
    public final Input<Integer> rightDriveClicks = addDashboardInput("rightDriveClicks", 0);

    public final Input<Double> leftDriveRate = addDashboardInput("leftDriveRate", 0.0);
    public final Input<Double> rightDriveRate = addDashboardInput("rightDriveRate", 0.0);

    public final Input<Double> totalCurrent = addDashboardInput("Current Drawn",0.0);

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
    
    public final Input<Boolean> isClamped = addDashboardInput("Is Clamped", false);
    
    public enum AutonMode {
        OFF,
        ROTATE_LEFT_360,
        ROTATE_RIGHT_360,
        FORWARD_6,
        BACKWARD_6,
        DEMO_NEW_AUTON,
        FORWARD_TEST,
        KINEMATIC_FORWARD,
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