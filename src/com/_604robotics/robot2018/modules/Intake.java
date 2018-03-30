package com._604robotics.robot2018.modules;

import com._604robotics.robot2018.constants.Calibration;
import com._604robotics.robot2018.constants.Ports;
import com._604robotics.robotnik.Action;
import com._604robotics.robotnik.Input;
import com._604robotics.robotnik.Module;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.VictorSP;

// TODO: Use current mode?
public class Intake extends Module {
	
    private WPI_TalonSRX innerMotorA = new WPI_TalonSRX(Ports.INTAKE_INNER_MOTOR_A); // right
    private WPI_TalonSRX innerMotorB = new WPI_TalonSRX(Ports.INTAKE_INNER_MOTOR_B);
    private VictorSP outerMotorA = new VictorSP(Ports.INTAKE_OUTER_MOTOR_A);
    private VictorSP outerMotorB = new VictorSP(Ports.INTAKE_OUTER_MOTOR_B);
	
    public Action run = new Run();
	public Action idle = new Idle();
        
    public class Run extends Action {
    	public final Input<Double> runPower;
    	
    	public Run() {
    		this(0);
    	}
    	
    	public Run(double power) {
    		super(Intake.this, Run.class);
    		runPower = addInput("Run Power", power, true);
    	}
    	
    	@Override
    	public void run() {
    		innerMotorA.set(runPower.get());
            innerMotorB.set(runPower.get());
    		outerMotorA.set(runPower.get());
    		outerMotorB.set(runPower.get());
    	}
    }
    
    public class Passive extends Action {
        public Passive() {
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
    
	public class Idle extends Action {
		public Idle() {
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
	
	public Intake () {
        super(Intake.class);
        innerMotorA.setInverted(true);
        innerMotorB.setInverted(true);
        outerMotorB.setInverted(true);
        setDefaultAction(idle);
    }
	
}
