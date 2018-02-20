package com._604robotics.robot2018.modules;

import com._604robotics.robot2018.constants.Ports;
import com._604robotics.robotnik.Action;
import com._604robotics.robotnik.Module;
import com._604robotics.robotnik.Output;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class Clamp extends Module {
	private final DoubleSolenoid solenoid = new DoubleSolenoid(Ports.CLAMP_A, Ports.CLAMP_B);
	
	private boolean clamping = true;
	
	public Output<Boolean> isClamped = addOutput("Clamping", this::isClamped);
	
	public boolean isClamped() {
		return !clamping;
	}
	
	public Action retract = new Retract();
	public Action extend = new Extend();
	
	public class Retract extends Action {
		public Retract() {
			super(Clamp.this, Retract.class);
		}
		
		@Override
		public void run() {
			solenoid.set(Value.kReverse);
			clamping = false;
		}
	}
	
	public class Extend extends Action {
		public Extend() {
			super(Clamp.this, Extend.class);
		}
		
		@Override
		public void run() {
			solenoid.set(Value.kForward);
			clamping = true;
		}
	}
	
	public Clamp() {
		super(Clamp.class);
		this.setDefaultAction(extend);
	}
}
