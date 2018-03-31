package com._604robotics.robotnik.prefabs.coordinators;

import com._604robotics.robotnik.Coordinator;
import com._604robotics.robotnik.prefabs.flow.SmartTimer;

public class TimeLimitCoordinator extends SleepCoordinator {
	private final Coordinator child;

	public TimeLimitCoordinator (double time, Coordinator child) {
	    super(time);
	    this.child = child;
	}

	@Override
	protected void begin() {
	    super.begin();
	    child.start();
	}

	@Override
	protected boolean run() {
		return super.run() && child.execute();
	}

	@Override
	protected void end() {
		child.stop();
	    super.end();
	}
}
