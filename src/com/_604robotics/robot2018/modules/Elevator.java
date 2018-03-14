package com._604robotics.robot2018.modules;

import com._604robotics.robot2018.constants.Calibration;
import com._604robotics.robot2018.constants.Ports;
import com._604robotics.robotnik.Action;
import com._604robotics.robotnik.Input;
import com._604robotics.robotnik.Module;
import com._604robotics.robotnik.Output;
import com._604robotics.robotnik.prefabs.controller.ClampedIntegralPIDController;
import com._604robotics.robotnik.prefabs.devices.TalonPWMEncoder;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class Elevator extends Module {

    private WPI_TalonSRX motorA = new WPI_TalonSRX(Ports.ELEVATOR_MOTOR_A);
    private WPI_TalonSRX motorB = new WPI_TalonSRX(Ports.ELEVATOR_MOTOR_B);
    public TalonPWMEncoder encoder = new TalonPWMEncoder(motorB);

    public double persistent = 0;
    
    public final Setpoint setpoint = new Setpoint();
    public final PersistentSetpoint persistentSetpoint = new PersistentSetpoint();
    
    public final Output<Double> encoderRate = addOutput("Elevator Rate", encoder::getVelocity);
    public final Output<Double> encoderClicks = addOutput("Elevator Clicks", encoder::getPosition);

    public boolean raised = false;
    public boolean holding = true;
    public double power = 0;

    public final Output<Boolean> getHolding = addOutput("Holding", this::getHolding);
    public final Output<Double> getPower = addOutput("Power", this::getPower);

    private final ClampedIntegralPIDController pid;
    
    public final Output<Double> pidError;
    
    public final Input<Boolean> clear = addInput("Clear", false);
    public final Output<Boolean> elevatorRaised = addOutput("Elevator Raised", this::getRaised);
    
    public boolean getRaised() {
    	if( encoder.getPosition() > Calibration.ELEVATOR_BUMPER_CLEAR ) {
    		System.out.println("Elevator Raised");
    	}
    	return encoder.getPosition() > Calibration.ELEVATOR_BUMPER_CLEAR;
    }
    
    public boolean getHolding() {
        return holding;
    }

    public double getPower() {
        return power;
    }
    
    public void resetIntegral(double sum) {
        pid.setErrorSum(sum);
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
        public void begin() {
        	setDefaultAction(setpoint);
        }
        
        @Override
        public void run () {
        	System.out.println("move");
            holding = false;
            power = liftPower.get();
            motorA.set(power);
            
            if( encoder.getPosition() < -Calibration.ELEVATOR_RESET_TOLERANCE ) {
            	encoder.zero();
            	System.out.println("Elevator zeroed.");
            }
            if( encoder.getPosition() > Calibration.ELEVATOR_HIGH_TARGET + Calibration.ELEVATOR_RESET_TOLERANCE ) {
            	encoder.setOffset(encoder.getPosition() - Calibration.ELEVATOR_HIGH_TARGET);
            	System.out.println("Elevator zeroed.");
            }
        }
    }

    public class Setpoint extends Action {
        public final Input<Double> target_clicks;

        public Setpoint() {
            this(0);
        }

        public Setpoint(double clicks) {
            super(Elevator.this, Setpoint.class);
            target_clicks = addInput("Target Elevator Clicks", clicks, true);
        }

        @Override
        public void begin() {
            pid.enable();
            holding = false;
            setDefaultAction(setpoint);
        }
        @Override
        public void run () {
        	System.out.println("setpoint enabled");
        	if( clear.get() ) {
        		System.out.println("Clear Active");
        	}
        	if( clear.get() && encoder.getPosition() < Calibration.ELEVATOR_BUMPER_CLEAR ) {
        		System.out.println("Raising");
        		target_clicks.set(Calibration.ELEVATOR_BUMPER_CLEAR);
        	}
            pid.setSetpoint(target_clicks.get());
        }
        @Override
        public void end () {
            pid.disable();
        }
    }

    public class SetPersistent extends Action {
        public final Input<Double> target_clicks;
    	
    	public SetPersistent(double target) {
    		super(Elevator.this, PersistentSetpoint.class);
    		target_clicks = addInput("Persistent Setpoint", target, true);
    	}
    	
    	@Override
    	public void begin() {
    		persistent = target_clicks.get();
    		setDefaultAction(persistentSetpoint);
    	}
    }
    
    public class PersistentSetpoint extends Action {
    	public PersistentSetpoint() {
    		super(Elevator.this, PersistentSetpoint.class);
    	}
    	
    	@Override
    	public void begin() {
    		pid.enable();
    	}
    	
    	@Override
    	public void run() {
    		System.out.println("persistent enabled");
        	if( clear.get() && encoder.getPosition() < Calibration.ELEVATOR_BUMPER_CLEAR ) {
        		System.out.println("clear enabled");
        		pid.setSetpoint(Calibration.ELEVATOR_BUMPER_CLEAR);
        	} else {
                pid.setSetpoint(persistent);
        	}
    	}
    	
    	@Override
    	public void end() {
    		pid.disable();
    	}
    }
    
    public Elevator() {
        super(Elevator.class);
        encoder.setInverted(false);
        encoder.setOffset(Calibration.ELEVATOR_ENCODER_ZERO);
        motorA.setInverted(true);
        motorB.setInverted(true);
        motorB.set(ControlMode.Follower,Ports.ELEVATOR_MOTOR_A);
        pid = new ClampedIntegralPIDController(Calibration.ELEVATOR_P,
                Calibration.ELEVATOR_I,
                Calibration.ELEVATOR_D,
                encoder,
                motorA,
                Calibration.ELEVATOR_PID_PERIOD);
        pid.setAbsoluteTolerance(Calibration.ELEVATOR_CLICK_TOLERANCE);
        pidError = addOutput("Elevator PID Error", pid::getError);
        pid.setIntegralLimits(Calibration.ELEVATOR_MIN_SUM, Calibration.ELEVATOR_MAX_SUM);
        pid.setOutputRange(Calibration.ELEVATOR_MIN_SPEED, Calibration.ELEVATOR_MAX_SPEED);
        //setpoint.target_clicks.set(encoder.getPosition());
        setDefaultAction(setpoint);
    }
}
