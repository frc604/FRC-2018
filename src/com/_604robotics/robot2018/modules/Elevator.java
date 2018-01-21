package com._604robotics.robot2018.modules;

import com._604robotics.robot2018.constants.Calibration;
import com._604robotics.robot2018.constants.Ports;
import com._604robotics.robotnik.Module;
import com._604robotics.robotnik.Output;
import com._604robotics.robotnik.prefabs.controller.AntiWindupPIDController;
import com._604robotics.robotnik.prefabs.devices.RateLimitedMotor;

import edu.wpi.first.wpilibj.CounterBase;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Elevator extends Module {
    private final Encoder encoder = new Encoder(Ports.ELEVATOR_ENCODER_A,
            Ports.ELEVATOR_ENCODER_B,
            true,
            CounterBase.EncodingType.k4X);
    
    private final Spark motor = new Spark(Ports.ELEVATOR_MOTOR);
    
    private final RateLimitedMotor rateLimitedMotor = new RateLimitedMotor(encoder, Calibration.ELEVATOR_RATE_TARGET, Calibration.ELEVATOR_RATE_TOLERANCE, motor);
    
    private final AntiWindupPIDController pid = 
            new AntiWindupPIDController(Calibration.ELEVATOR_P, Calibration.ELEVATOR_I, Calibration.ELEVATOR_A, Calibration.ELEVATOR_C, Calibration.ELEVATOR_D, encoder, rateLimitedMotor);
    
    public final Output<Integer> elevatorClicks = addOutput("Elevator Clicks", encoder::get);
    public final Output<Double> elevatorRate = addOutput("Elevator Rate", encoder::getRate);
    
    /* Begin jankiest part in this entire robot code */
    
    private  Timer timer = new Timer();
    private boolean timing = false;
    
    public final Boolean atElevatorTarget() {
        if (pid.isEnabled() && pid.onTarget()) {
            if (!timing) {
                timing = true;
                timer.start();
            }
            
            return timer.get() >= 0.25;
        } else {
            if (timing) {
                timing = false;
                
                timer.stop();
                timer.reset();
            }
            
            return false;
        }
    }
    
    public final Output<Boolean> atTarget = addOutput("At Target", this::atElevatorTarget);
    /* End jankiest part in this entire robot code */
    
    public Elevator() {
        super(Elevator.class);

        SmartDashboard.putData("Elevator PID", pid);
        pid.setAbsoluteTolerance(12);
        encoder.setDistancePerPulse(1);
        encoder.reset();
        rateLimitedMotor.setRampRate(0.01);
    }
}
