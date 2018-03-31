package com._604robotics.robotnik.prefabs.modules;

import com._604robotics.robotnik.Action;
import com._604robotics.robotnik.Module;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class Shifter extends Module {
	private final DoubleSolenoid solenoid;

	public class Gear extends Action {
		private Value value;

		private Gear (Value value) {
			super(Shifter.this, Gear.class);
			this.value = value;
		}
		
		@Override
        public void begin() {
			solenoid.set(value);
		}
	}
	
	public final Gear lowGear = new Gear(Value.kReverse);
	public final Gear highGear = new Gear(Value.kForward);

	public Shifter (int highGearSolenoid, int lowGearSolenoid) {
		super(Shifter.class);

		solenoid = new DoubleSolenoid(highGearSolenoid, lowGearSolenoid);

		setDefaultAction(lowGear);
	}
}
