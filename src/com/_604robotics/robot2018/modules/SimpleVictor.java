package com._604robotics.robot2018.modules;

import com._604robotics.robotnik.Action;
import com._604robotics.robotnik.Input;
import com._604robotics.robotnik.Module;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;


public class SimpleVictor extends Module {
	public WPI_VictorSPX motor = new WPI_VictorSPX(11);

	public final Action move = new Move();
	public final Action idle = new Idle();
	
	public class Move extends Action {
        public final Input<Double> power;

        public Move() {
            this(0);
        }

        public Move (double power) {
            super(SimpleVictor.this, Move.class);
            this.power = addInput("Power", power, true);
        }

        @Override
        public void run () {
            motor.set(power.get());
        }
    }
	
	public class Idle extends Action {
		public Idle() {
			super(SimpleVictor.this, Idle.class);
		}
		
		@Override
		public void run() {
			motor.set(0);
		}
	}
	
	public SimpleVictor() {
        super(SimpleVictor.class);
        setDefaultAction(idle);
    }
}
