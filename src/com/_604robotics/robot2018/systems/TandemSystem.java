package com._604robotics.robot2018.systems;

import com._604robotics.robot2018.Robot2018;
import com._604robotics.robotnik.Coordinator;

public class TandemSystem extends Coordinator {
	private final Robot2018 robot;

    public TandemSystem (Robot2018 robot) {
        this.robot = robot;
    }
    
    @Override
    public boolean run() {
    	robot.arm.elevatorRaised.set(robot.elevator.elevatorRaised.get());
    	robot.elevator.clear.set(robot.arm.clear.get());
    	return true;
    }
}
