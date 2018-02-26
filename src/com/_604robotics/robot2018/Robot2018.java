package com._604robotics.robot2018;

import com._604robotics.robot2018.constants.Ports;
import com._604robotics.robot2018.modes.*;
import com._604robotics.robot2018.modules.*;
import com._604robotics.robot2018.systems.*;
import com._604robotics.robotnik.Robot;
import com._604robotics.robotnik.prefabs.modules.PowerMonitor;
import com._604robotics.robotnik.prefabs.modules.Shifter;

public class Robot2018 extends Robot {
    public final Dashboard dashboard = addModule(new Dashboard());
    public final Drive drive = addModule(new Drive());
    public final Elevator elevator = addModule(new Elevator());
    public final Intake intake = addModule(new Intake());
    public final Shifter shifter = addModule(new Shifter(Ports.SHIFTER_A, Ports.SHIFTER_B));
    public final Clamp clamp = addModule(new Clamp());
    public final Arm arm = addModule(new Arm());
    public final PowerMonitor powermonitor = addModule(new PowerMonitor(Ports.PDP_MODULE, Ports.COMPRESSOR));
    
    public Robot2018() {
        setAutonomousMode(new AutonomousMode(this));
        setTeleopMode(new TeleopMode(this));
                
        addSystem(DashboardSystem.class, new DashboardSystem(this));
    }
}
