package com._604robotics.robot2018.modules;

import com._604robotics.robot2018.constants.Ports;
import com._604robotics.robotnik.Action;
import com._604robotics.robotnik.Module;
import com._604robotics.robotnik.Output;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class Clamp extends Module {
	private final DoubleSolenoid solenoid = new DoubleSolenoid(Ports.CLAMP_A, Ports.CLAMP_B);

	public class Engage extends Action {
		private Engage () {
			super(Clamp.this, Engage.class);
		}
		
		@Override
		public void run() {
			solenoid.set(Value.kReverse);
		}
	}
	public final Engage engage = new Engage();

	public class Release extends Action {
		private Release () {
			super(Clamp.this, Release.class);
		}
		
		@Override
		public void run() {
			solenoid.set(Value.kForward);
		}
	}
	public final Release release = new Release();

	public final Output<Boolean> clamped = addOutput("clamped", engage::isRunning);

	public Clamp() {
		super(Clamp.class);
		setDefaultAction(engage);
	}
}
