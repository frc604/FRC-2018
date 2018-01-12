package com._604robotics.robot2018;

import com._604robotics.robot2018.systems.DashboardSystem;
import com._604robotics.robot2018.constants.Ports;
import com._604robotics.robot2018.modes.AutonomousMode;
import com._604robotics.robot2018.modes.TeleopMode;
import com._604robotics.robot2018.modules.Dashboard;
import com._604robotics.robot2018.modules.Drive;
import com._604robotics.robotnik.Coordinator;
import com._604robotics.robotnik.Robot;
import com._604robotics.robotnik.prefabs.modules.PowerMonitor;

public class Robot2018 extends Robot {
    public final Dashboard dashboard = addModule(new Dashboard());
    public final Drive drive = addModule(new Drive());
    public final PowerMonitor powermonitor = addModule(new PowerMonitor(Ports.PDP_MODULE, Ports.COMPRESSOR));
    
    public Robot2018() {
        Coordinator auton = new AutonomousMode(this);
        Coordinator teleop = new TeleopMode(this);
        setAutonomousMode(auton);
        setTeleopMode(teleop);
        setTestMode(teleop);
        
        addSystem(DashboardSystem.class, new DashboardSystem(this));
    }
}
