package com._604robotics.robot2018.modules;

import com._604robotics.robot2018.constants.Ports;
import com._604robotics.robotnik.Action;
import com._604robotics.robotnik.Input;
import com._604robotics.robotnik.Module;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class Intake extends Module {
	
    public WPI_TalonSRX motorA = new WPI_TalonSRX(Ports.INTAKE_MOTOR_A);
    public WPI_TalonSRX motorB = new WPI_TalonSRX(Ports.INTAKE_MOTOR_B);
	
    public Action run = new Suck();
    
    public class Suck extends Action {
    	public final Input<Double> suckPower;
    	
    	public Suck() {
    		this(0);
    	}
    	
    	public Suck(double power) {
    		super(Intake.this, Suck.class);
    		suckPower = addInput("Suck Power", power, true);
    	}
    	
    	@Override
    	public void run() {
    		motorA.set(suckPower.get());
    		motorB.set(suckPower.get());
    	}
    }
    
    public class Spit extends Action {
    	public final Input<Double> spitPower;
    	
    	public Spit() {
    		this(0);
    	}
    	
    	public Spit(double power) {
    		super(Intake.this, Suck.class);
    		spitPower = addInput("Spit Power", power, true);
    	}
    	
    	@Override
    	public void run() {
    		motorA.set(-spitPower.get());
    		motorB.set(-spitPower.get());
    	}
    }
    
	public Action idle = new Idle();
	
	public class Idle extends Action {
		public Idle() {
			super(Intake.this, Idle.class);
		}
		
		@Override
		public void run() {
			
		}
	}
	
	public Intake () {
        super(Intake.class);
        setDefaultAction(idle);
    }
	
}
