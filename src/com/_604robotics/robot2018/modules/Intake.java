package com._604robotics.robot2018.modules;

import com._604robotics.robot2018.constants.Calibration;
import com._604robotics.robot2018.constants.Ports;
import com._604robotics.robotnik.Action;
import com._604robotics.robotnik.Input;
import com._604robotics.robotnik.Module;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.VictorSP;

// TODO: Use current mode?
public class Intake extends Module {
	
    private VictorSP motorA = new VictorSP(Ports.INTAKE_OUTER_MOTOR_A);
    private VictorSP motorB = new VictorSP(Ports.INTAKE_OUTER_MOTOR_B);
    private WPI_VictorSPX outerMotorA = new WPI_VictorSPX(Ports.INTAKE_INNER_MOTOR_A);
    private WPI_VictorSPX outerMotorB = new WPI_VictorSPX(Ports.INTAKE_INNER_MOTOR_B);
	
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
    		motorA.set(runPower.get());
    		motorB.set(runPower.get());
    		outerMotorA.set(runPower.get());
    	}
    }
    	
	public class Idle extends Action {
		public Idle() {
			super(Intake.this, Idle.class);
		}
		
		@Override
		public void run() {
			motorA.set(Calibration.INTAKE_PASSIVE_POWER);
			motorB.set(Calibration.INTAKE_PASSIVE_POWER);
			outerMotorA.set(Calibration.INTAKE_PASSIVE_POWER);
		}
	}
	
	public Intake () {
        super(Intake.class);
        motorB.setInverted(true);
        outerMotorB.setInverted(true);
        outerMotorB.set(ControlMode.Follower, Ports.INTAKE_OUTER_MOTOR_A);
        setDefaultAction(idle);
    }
	
}
