package com._604robotics.robot2018.systems;

import com._604robotics.robot2018.Robot2018;
import com._604robotics.robotnik.Coordinator;
import com._604robotics.robotnik.prefabs.modules.PowerMonitor;

public class DashboardSystem extends Coordinator {
    private final Robot2018 robot;

    public DashboardSystem (Robot2018 robot) {
        this.robot = robot;
    }

    @Override
    public boolean run () {
        //@Janky("Will soon be replacable with initDashboardSendables")
        robot.dashboard.leftDriveClicks.set(robot.drive.leftClicks.get());
        robot.dashboard.rightDriveClicks.set(robot.drive.rightClicks.get());
        robot.dashboard.leftDriveRate.set(robot.drive.leftClickRate.get());
        robot.dashboard.rightDriveRate.set(robot.drive.rightClickRate.get());
        //robot.dashboard.gyroAngle.set(robot.drive.gyroAngle.get());
        robot.dashboard.totalCurrent.set(PowerMonitor.totalPortCurrent.get());        
        return true;
    }
}
