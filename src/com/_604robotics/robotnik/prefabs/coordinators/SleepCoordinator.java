package com._604robotics.robotnik.prefabs.coordinators;

import com._604robotics.robotnik.Coordinator;
import com._604robotics.robotnik.prefabs.flow.SmartTimer;

/**
 * A coordinator that implements a pause for a specified time.
 */
public class SleepCoordinator extends Coordinator {

	private SmartTimer sleepTimer = new SmartTimer();
	private final double sleepTime;

	public SleepCoordinator(double time) {
		if (time<=0) {
			throw new IllegalArgumentException("Must sleep for a positive amount of time!");
		}
		sleepTime=time;
	}

	@Override
	protected void begin() {
		sleepTimer.start();
	}

	@Override
	protected boolean run() {
		return !sleepTimer.hasReachedTime(sleepTime);
	}

	@Override
	protected void end() {
		sleepTimer.stopAndReset();
	}

}
