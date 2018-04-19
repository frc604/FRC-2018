package com._604robotics.robot2018.systems;

import com._604robotics.robot2018.Robot2018;
import com._604robotics.robot2018.constants.Calibration;
import com._604robotics.robotnik.Coordinator;

public class DashboardSystem extends Coordinator {
    private final Robot2018 robot;

    public DashboardSystem (Robot2018 robot) {
        this.robot = robot;
    }

    @Override
    public boolean run () {
        robot.dashboard.leftDriveClicks.set(robot.drive.leftClicks.get());
        robot.dashboard.rightDriveClicks.set(robot.drive.rightClicks.get());
        robot.dashboard.leftDriveRate.set(robot.drive.leftClickRate.get());
        robot.dashboard.rightDriveRate.set(robot.drive.rightClickRate.get());

        //robot.dashboard.totalCurrent.set(robot.powermonitor.totalPortCurrent.get());
        
        robot.dashboard.elevatorRate.set(robot.elevator.encoderRate.get());
        robot.dashboard.elevatorClicks.set(robot.elevator.encoderClicks.get());
        
        //robot.dashboard.holding.set(robot.elevator.getHolding.get());
        //robot.dashboard.power.set(robot.elevator.getPower.get());
        
        robot.dashboard.armRate.set(robot.arm.encoderRate.get());
        robot.dashboard.armClicks.set(robot.arm.encoderClicks.get());
        
        robot.dashboard.isClamped.set(robot.clamp.isClamped.get() ? "CLAMPED" : "NOT CLAMPED");
        robot.dashboard.clampLightA.set(robot.clamp.isClamped.get());
        robot.dashboard.clampLightB.set(robot.clamp.isClamped.get());
        robot.dashboard.armEncoderStatus.set(robot.arm.encoderClicks.get()>4700 || robot.arm.encoderClicks.get() < -2500);
        robot.dashboard.limitPressed.set(robot.arm.getBottomLimit());
        //robot.powermonitor.initDashboardSendables();
        
        switch( robot.dashboard.marionetteRecorder.get() ) {
            case MANUAL:
                robot.dashboard.writeFile.set(robot.dashboard.filePrefix.get() + robot.dashboard.marionetteFile.get());
                break;
            case SWITCH_LEFT:
                robot.dashboard.writeFile.set(robot.dashboard.filePrefix.get() + Calibration.SWITCH_LEFT_FILENAME);
                break;
            case SWITCH_RIGHT:
                robot.dashboard.writeFile.set(robot.dashboard.filePrefix.get() + Calibration.SWITCH_RIGHT_FILENAME);
                break;
            case SCALE_LEFT:
                robot.dashboard.writeFile.set(robot.dashboard.filePrefix.get() + Calibration.SCALE_LEFT_FILENAME);
                break;
            case SCALE_RIGHT:
                robot.dashboard.writeFile.set(robot.dashboard.filePrefix.get() + Calibration.SCALE_RIGHT_FILENAME);
                break;
            default:
                break;
        }
        
        switch( robot.dashboard.marionetteOutput.get() ) {
            case MANUAL:
                robot.dashboard.primaryReadFile.set(robot.dashboard.filePrefix.get() + robot.dashboard.marionetteFile.get());
                break;
            case SWITCH:
                robot.dashboard.primaryReadFile.set(robot.dashboard.filePrefix.get() + Calibration.SWITCH_LEFT_FILENAME);
                robot.dashboard.secondaryReadFile.set(robot.dashboard.filePrefix.get() + Calibration.SWITCH_RIGHT_FILENAME);
                break;
            case SCALE_LEFT:
                robot.dashboard.primaryReadFile.set(robot.dashboard.filePrefix.get() + Calibration.SCALE_LEFT_FILENAME);
                break;
            case SCALE_RIGHT:
                robot.dashboard.primaryReadFile.set(robot.dashboard.filePrefix.get() + Calibration.SCALE_RIGHT_FILENAME);
                break;
            default:
                break;
        }
        return true;
    }
}
