package com._604robotics.robot2018.modules;

import com._604robotics.robot2018.constants.Ports;
import com._604robotics.robotnik.Action;
import com._604robotics.robotnik.Input;
import com._604robotics.robotnik.Module;
import com._604robotics.robotnik.prefabs.devices.HoldMotor;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Spark;

public class Elevator extends Module {
	
	public Spark motor = new Spark(Ports.ELEVATOR_MOTOR);
	public Encoder encoder = new Encoder(Ports.ELEVATOR_ENCODER_A, Ports.ELEVATOR_ENCODER_B);
	public HoldMotor manualControl = new HoldMotor(motor, encoder);
	
	public final Action hold = new Hold();
	
	public class Hold extends Action {
        public Hold () {
            super(Elevator.this, Hold.class);
        }

        @Override
        public void run () {
            manualControl.pidWrite(0);
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
			manualControl.pidWrite(liftPower.get());
		}
	}
	
	public Elevator() {
        super(Elevator.class);
        setDefaultAction(hold);
	}
}
