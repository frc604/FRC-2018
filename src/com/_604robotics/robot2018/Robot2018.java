package com._604robotics.robot2018;

import com._604robotics.robot2018.constants.Ports;
import com._604robotics.robot2018.modes.AutonomousMode;
import com._604robotics.robot2018.modes.TeleopMode;
import com._604robotics.robot2018.modules.Arm;
import com._604robotics.robot2018.modules.Camera;
import com._604robotics.robot2018.modules.Clamp;
import com._604robotics.robot2018.modules.Dashboard;
import com._604robotics.robot2018.modules.Drive;
import com._604robotics.robot2018.modules.Elevator;
import com._604robotics.robot2018.modules.Intake;
import com._604robotics.robot2018.modules.PixyTest;
import com._604robotics.robot2018.systems.DashboardSystem;
import com._604robotics.robotnik.Robot;
import com._604robotics.robotnik.prefabs.modules.Shifter;

public class Robot2018 extends Robot {
    public final Dashboard dashboard = addModule(new Dashboard());
    public final Drive drive = addModule(new Drive());
    public final Elevator elevator = addModule(new Elevator());
    public final Intake intake = addModule(new Intake());
    public final Shifter shifter = addModule(new Shifter(Ports.SHIFTER_A, Ports.SHIFTER_B));
    public final Clamp clamp = addModule(new Clamp());
    public final Arm arm = addModule(new Arm());
    public final Camera camera = addModule(new Camera());
    public final PixyTest pixyTest = addModule(new PixyTest());
    //public final PowerMonitor powermonitor = addModule(new PowerMonitor(Ports.PDP_MODULE, Ports.COMPRESSOR));
    
    public Robot2018() {
        setAutonomousMode(new AutonomousMode(this));
        setTeleopMode(new TeleopMode(this));
                
        addSystem(DashboardSystem.class, new DashboardSystem(this));
    }
}
