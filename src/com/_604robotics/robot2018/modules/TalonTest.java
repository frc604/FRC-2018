package com._604robotics.robot2018.modules;

import com._604robotics.robot2018.constants.Ports;
import com._604robotics.robotnik.Action;
import com._604robotics.robotnik.Input;
import com._604robotics.robotnik.Module;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class TalonTest extends Module {
    public WPI_TalonSRX motorA = new WPI_TalonSRX(Ports.ELEVATOR_MOTOR_A);
    public WPI_TalonSRX motorB = new WPI_TalonSRX(Ports.ELEVATOR_MOTOR_B);

    public final Action move = new Move();
    
    public class Move extends Action {
        public final Input<Double> liftPower;

        public Move() {
            this(0);
        }

        public Move (double power) {
            super(TalonTest.this, Move.class);
            liftPower = addInput("Lift Power", power, true);
        }

        @Override
        public void run () {
            motorA.set(liftPower.get());
            motorB.set(liftPower.get());
        }
    }
    
    public final Action idle = new Idle();
    
	public class Idle extends Action {
		public Idle() {
			super(TalonTest.this, Idle.class);
		}
		
		@Override
		public void run() {
			motorA.set(0);
			motorB.set(0);
		}
	}
	
	public TalonTest() {
		super(TalonTest.class);
        setDefaultAction(idle);
	}
}
