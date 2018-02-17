package com._604robotics.robot2018.systems;

import com._604robotics.robot2018.Robot2018;
import com._604robotics.robotnik.Coordinator;

public class DashboardSystem extends Coordinator {
    private final Robot2018 robot;

    public DashboardSystem (Robot2018 robot) {
        this.robot = robot;
    }

    @Override
    public boolean run () {
        //@Janky("Will soon be replacable with initDashboardSendables")
        robot.drive.updateDashboardSendables();
        robot.dashboard.leftDriveClicks.set(robot.drive.leftClicks.get());
        robot.dashboard.rightDriveClicks.set(robot.drive.rightClicks.get());
        robot.dashboard.leftDriveRate.set(robot.drive.leftClickRate.get());
        robot.dashboard.rightDriveRate.set(robot.drive.rightClickRate.get());
        //robot.dashboard.gyroAngle.set(robot.drive.gyroAngle.get());
        //robot.dashboard.totalCurrent.set(robot.powermonitor.totalPortCurrent.get());  
        
        /*robot.dashboard.elevatorOffset.set(robot.elevator.getOffset.get());
        robot.dashboard.elevatorUpwardsRange.set(robot.elevator.getUpwardsRange.get());
        robot.dashboard.elevatorDownwardsRange.set(robot.elevator.getDownwardsRange.get());
        robot.dashboard.elevatorFailsafed.set(robot.elevator.getFailsafe.get());*/
        
        robot.dashboard.elevatorRate.set(robot.elevator.encoderRate.get());
        robot.dashboard.elevatorClicks.set(robot.elevator.encoderClicks.get());
        
        //robot.dashboard.holding.set(robot.elevator.getHolding.get());
        //robot.dashboard.power.set(robot.elevator.getPower.get());
        
        robot.dashboard.armRate.set(robot.arm.encoderRate.get());
        robot.dashboard.armClicks.set(robot.arm.encoderClicks.get());
        
        robot.dashboard.isClamped.set(robot.clamp.isClamped.get());
        //robot.powermonitor.initDashboardSendables();
        return true;
    }
}
