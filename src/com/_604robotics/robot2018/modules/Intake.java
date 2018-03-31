package com._604robotics.robot2018.modules;

import com._604robotics.robot2018.constants.Ports;
import com._604robotics.robotnik.Action;
import com._604robotics.robotnik.Input;
import com._604robotics.robotnik.Module;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.VictorSP;

public class Intake extends Module {
    private final WPI_TalonSRX innerMotorA = new WPI_TalonSRX(Ports.INTAKE_INNER_MOTOR_A); // right
    private final WPI_TalonSRX innerMotorB = new WPI_TalonSRX(Ports.INTAKE_INNER_MOTOR_B);

    private final VictorSP outerMotorA = new VictorSP(Ports.INTAKE_OUTER_MOTOR_A);
    private final VictorSP outerMotorB = new VictorSP(Ports.INTAKE_OUTER_MOTOR_B);

	public class Idle extends Action {
		private Idle() {
			super(Intake.this, Idle.class);
		}

		@Override
		public void run() {
			innerMotorA.set(0);
			outerMotorA.set(0);
			outerMotorB.set(0);
		}
	}
	public final Idle idle = new Idle();

    public class Run extends Action {
    	public final Input<Double> power;
    	
    	public Run () {
    		this(0);
    	}
    	
    	public Run (double defaultPower) {
    		super(Intake.this, Run.class);
    		power = addInput("power", defaultPower);
    	}
    	
    	@Override
    	public void run() {
    		innerMotorA.set(power.get());
    		outerMotorA.set(power.get());
    		outerMotorB.set(power.get());
    	}
    }
    
	public Intake () {
        super(Intake.class);

        innerMotorA.setInverted(true);

        innerMotorB.setInverted(true);
		innerMotorB.set(ControlMode.Follower, Ports.INTAKE_INNER_MOTOR_A);

        outerMotorB.setInverted(true);

        setDefaultAction(idle);
    }
}
