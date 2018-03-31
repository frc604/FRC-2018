package com._604robotics.robotnik.prefabs.modules;

import com._604robotics.robotnik.Logger;
import com._604robotics.robotnik.Module;
import com._604robotics.robotnik.Output;
import com._604robotics.robotnik.prefabs.flow.Pulse;
import com._604robotics.robotnik.prefabs.flow.SmartTimer;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class PowerMonitor extends Module {

    private final PowerDistributionPanel panel;
    private final Compressor compressor;

    public static final int NUM_CURRENT_PORTS = 16;
    public Output<Double> totalPortCurrent;
    public Output<Double> totalCurrent;
    public Output<Double> batteryVoltage;

    // Specifying <Double> results in an error
    // Creating a special EmptyOutput class causes ArrayStoreExceptions
    @SuppressWarnings("unchecked")
    public Output<Double>[] currents = new Output[NUM_CURRENT_PORTS];
    public Output<Double> compCurrent;

    public static double voltageLimit = 7.5;

    // While measured values may change, the current limits on each PDP stays the same
    public static double[] currentLimit = {40, 40, 40, 40,
            20, 20, 20, 20,
            20, 20, 20, 20,
            40, 40, 40, 40};
    
    static {
        for (int i = 0; i < currentLimit.length; i++) {
            currentLimit[i] *= 0.8;
        }
    }

    private SmartTimer [] timerArray = new SmartTimer[NUM_CURRENT_PORTS];
    private SmartTimer voltageTimer = new SmartTimer();
    private Pulse voltagePulse = new Pulse();
    private Pulse [] pulseArray = new Pulse[NUM_CURRENT_PORTS];
    
    private static final Logger theLogger = new Logger(PowerMonitor.class);
    private void resetTimers() {
        for (int i=0;i<NUM_CURRENT_PORTS;i++) {
            timerArray[i]=new SmartTimer();
            timerArray[i].stopAndReset();
            pulseArray[i]=new Pulse();
            pulseArray[i].update(false);
        }
        voltageTimer.stopAndReset();
        voltagePulse.update(false);
    }

    public PowerMonitor(int PDPPortID, int compressorID) {
        super(PowerMonitor.class);
        // Initialize WPILib Objects
        panel = new PowerDistributionPanel(PDPPortID);
        compressor = new Compressor(compressorID);
        // Set up outputs
        compCurrent = addOutput("Compressor Current", 0d, () -> compressor.getCompressorCurrent());
        totalPortCurrent=addOutput("Total Port Current", 0d, () -> panel.getTotalCurrent());
        totalCurrent=addOutput("Total Current", 0d, () -> panel.getTotalCurrent()+compressor.getCompressorCurrent());
        batteryVoltage=addOutput("Battery Voltage", 0d, () -> panel.getVoltage());
        for (int port=0;port<NUM_CURRENT_PORTS;port++) {
            int proxy=port;
            Output<Double> tempOutput = addOutput("Port "+port+" Current", 0d, () -> panel.getCurrent(proxy));
            currents[proxy]=tempOutput;
        }
        for (Output<Double> test:currents) {
            if (test==null) {
                theLogger.log("WARN","null tempOutput");
            }
        }
    }
    
    public void initDashboardSendables() {
        SmartDashboard.putData("PDP", panel);
    }
    protected void begin() {
        resetTimers();
    }

    protected void run() {
        for (int i=0;i<NUM_CURRENT_PORTS;i++) {
            if (timerArray[i].isRunning()) {
                if ((timerArray[i].get()%0.5)<0.05 && (timerArray[i].get()>0.44)) {
                    theLogger.log("WARN", "Excess current of "+currents[i].get()+" through PDP Port "+i+"!");
                    theLogger.log("WARN", "Excess has continued for "+timerArray[i].get()+" seconds.");
                }
            }
        }
        if (voltageTimer.isRunning()) {
            if ((voltageTimer.get()%0.5)<0.05 && (voltageTimer.get()>0.44)) {
                theLogger.log("WARN", "Low voltage of "+batteryVoltage.get()+"V!");
                theLogger.log("WARN", "Voltage drop has continued for "+voltageTimer.get()+" seconds.");
            }
        }
        for (int i=0;i<NUM_CURRENT_PORTS;i++) {
            if (currents[i].get() > currentLimit[i]) {
                pulseArray[i].update(true);
                timerArray[i].startIfNotRunning();
                // Do not print start since such transients occur during normal operation
                if (pulseArray[i].isRisingEdge()) {
                    //theLogger.log("WARN", "Excess current started through PDP Port "+i+"!");
                }
            } else {
                pulseArray[i].update(false);
                timerArray[i].stop();
                // Print only when this is not a transient
                if (pulseArray[i].isFallingEdge() && timerArray[i].get()>0.4) {
                    theLogger.log("WARN", "Excess current ended through PDP Port "+i+" after "+timerArray[i].get()+" seconds.");
                }
                timerArray[i].reset();
            }
        }
        if (batteryVoltage.get()<voltageLimit) {
            voltagePulse.update(true);
            voltageTimer.startIfNotRunning();
            if (voltagePulse.isRisingEdge()) {
                theLogger.log("WARN", "Voltage has dropped below "+voltageLimit+"V!");
            }
        } else {
            voltagePulse.update(false);
            voltageTimer.stop();
            if (voltagePulse.isFallingEdge()) {
                theLogger.log("WARN", "Voltage has recovered above "+voltageLimit+"V!");
            }
            voltageTimer.reset();
        }
    }

    protected void end() {
        resetTimers();
    }
}
