package com._604robotics.robotnik.prefabs.modules;

import com._604robotics.robotnik.Action;
import com._604robotics.robotnik.Module;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class Shifter extends Module {

	private final DoubleSolenoid solenoid;

	public Shifter(int forwardSolenoid, int reverseSolenoid) {
		super(Shifter.class);
		this.solenoid=new DoubleSolenoid(forwardSolenoid,reverseSolenoid);
		setDefaultAction(idle);
	}
	
	private class Idle extends Action {
		public Idle() {
			super(Shifter.this,Idle.class);
		}
		@Override
        public void begin() {
			solenoid.set(Value.kReverse);
		}
	}
	private class SetGear extends Action {
		private Value currentState;
		public SetGear(Value initState) {
			super(Shifter.this, SetGear.class);
			currentState=initState;
		}
		
		@Override
        public void begin() {
			solenoid.set(currentState);
		}
	}
	
	public final Action idle = new Idle();
	public final Action lowGear=new SetGear(Value.kReverse);
	public final Action highGear=new SetGear(Value.kForward);
}
