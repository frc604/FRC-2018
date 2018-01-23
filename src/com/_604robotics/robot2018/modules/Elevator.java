package com._604robotics.robot2018.modules;

import com._604robotics.robot2018.constants.Ports;
import com._604robotics.robotnik.Action;
import com._604robotics.robotnik.Input;
import com._604robotics.robotnik.Module;
import com._604robotics.robotnik.Output;
import com._604robotics.robotnik.prefabs.devices.HoldMotor;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Spark;

public class Elevator extends Module {
	
	public Spark motor = new Spark(Ports.ELEVATOR_MOTOR);
	public Encoder encoder = new Encoder(Ports.ELEVATOR_ENCODER_A, Ports.ELEVATOR_ENCODER_B);
	public HoldMotor manualControl = new HoldMotor(motor, encoder);
	
	public final Action hold = new Hold();
	
	public final Output<Double> getOffset = addOutput("Elevator Offset", this::getOffset);
	public final Output<Double> getUpwardsRange = addOutput("Upwards Range", this::getUpwardsRange);
	public final Output<Double> getDownwardsRange = addOutput("Downwards Range", this::getDownwardsRange);
	public final Output<Boolean> getFailsafe = addOutput("Failsafed", this::getFailsafe);
	
	public final Output<Double> encoderRate = addOutput("Elevator Rate", encoder::getRate);
	public final Output<Integer> encoderClicks = addOutput("Elevator Clicks", encoder::get);
	
	public boolean holding = true;
	public double power = 0;
	
	public final Output<Boolean> getHolding = addOutput("Holding", this::getHolding);
	public final Output<Double> getPower = addOutput("Power", this::getPower);
	
	public boolean getHolding() {
		return holding;
	}
	
	public double getPower() {
		return power;
	}
	
	public double getOffset() {
		return manualControl.offset;
	}
	
	public double getUpwardsRange() {
		return manualControl.upwardsRange;
	}
	
	public double getDownwardsRange() {
		return manualControl.downwardsRange;
	}
	
	public boolean getFailsafe() {
		return manualControl.failsafed;
	}
	
	public class Hold extends Action {
        public Hold () {
            super(Elevator.this, Hold.class);
        }

        @Override
        public void run () {
        	holding = true;
            manualControl.hold();
        }
    }
	
	public class Move extends Action {
		public final Input<Double> liftPower;
		
		public Move() {
			this(0);
		}
		
		public Move (double power) {
			super(Elevator.this, Move.class);
			liftPower = addInput("Lift Power", power, true);
		}
		
		@Override
		public void run () {
			holding = false;
			power = liftPower.get();
			manualControl.set(liftPower.get());
		}
	}
	
	public Elevator() {
        super(Elevator.class);
        setDefaultAction(hold);
	}
}
