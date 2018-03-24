package com._604robotics.robot2018.systems;

import com._604robotics.robot2018.Robot2018;
import com._604robotics.robotnik.Coordinator;

import edu.wpi.first.wpilibj.DriverStation;

public class GameDataSystem extends Coordinator {
	private final Robot2018 robot;
	
	public GameDataSystem(Robot2018 robot) {
		this.robot = robot;
	}
	
	@Override
	public boolean run() {
		robot.dashboard.gameData.set(DriverStation.getInstance().getGameSpecificMessage());
		return true;
	}
}
