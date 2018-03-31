package com._604robotics.robot2018.modules;

import com._604robotics.robot2018.constants.Calibration;
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
			innerMotorB.set(0);
			outerMotorA.set(0);
			outerMotorB.set(0);
		}
	}
	public final Idle idle = new Idle();

	public class Passive extends Action {
		private Passive() {
			super(Intake.this, Passive.class);
		}

		@Override
		public void run() {
			innerMotorA.set(Calibration.INTAKE_PASSIVE_POWER);
			innerMotorB.set(Calibration.INTAKE_PASSIVE_POWER);

			outerMotorA.set(0);
			outerMotorB.set(0);
		}
	}
	public final Passive passive = new Passive();

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
			innerMotorB.set(power.get());

    		outerMotorA.set(power.get());
    		outerMotorB.set(power.get());
    	}
    }
    
	public Intake () {
        super(Intake.class);

        innerMotorA.setInverted(true);
		innerMotorA.setNeutralMode(com.ctre.phoenix.motorcontrol.NeutralMode.Coast);

        innerMotorB.setInverted(true);
		innerMotorB.setNeutralMode(com.ctre.phoenix.motorcontrol.NeutralMode.Coast);

        outerMotorB.setInverted(true);

        setDefaultAction(idle);
    }
}
