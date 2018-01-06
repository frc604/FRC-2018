package com._604robotics.robotnik.prefabs.modules;

import com._604robotics.robotnik.Logger;
import com._604robotics.robotnik.Module;
import com._604robotics.robotnik.Output;
import com._604robotics.robotnik.prefabs.flow.SmartTimer;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PowerDistributionPanel;

public class PowerMonitor extends Module {

    private final PowerDistributionPanel panel;
    private final Compressor compressor;
    public static Output<Double> totalPortCurrent;
    public static Output<Double> totalCurrent;
    public static Output<Double> batteryVoltage;

    // Specifying <Double> results in an error
    // Creating a special EmptyOutput class causes ArrayStoreExceptions
    @SuppressWarnings("unchecked")
    public static Output<Double> [] currents = new Output[16];
    public static Output<Double> compCurrent;
    public static double [] currentLimit = {40, 40, 40, 40,
                                     20, 20, 20, 20,
                                     20, 20, 20, 20,
                                     40, 40, 40, 40};

    static {
        for (int i=0;i<currentLimit.length;i++) {
            currentLimit[i]*=0.8;
        }
    }

    private static final Logger theLogger = new Logger(PowerMonitor.class);
    private final SmartTimer iterTimer = new SmartTimer();

    public PowerMonitor(int PDPPortID, int compressorID) {
        super(PowerMonitor.class);
        panel = new PowerDistributionPanel(PDPPortID);
        compressor = new Compressor(compressorID);
        compCurrent = addOutput("Compressor Current", () -> compressor.getCompressorCurrent());
        totalPortCurrent=addOutput("Total Port Current", () -> panel.getTotalCurrent());
        totalCurrent=addOutput("Total Current", () -> panel.getTotalCurrent()+compressor.getCompressorCurrent());
        batteryVoltage=addOutput("Battery Voltage", () -> panel.getVoltage());
        for (int port=0;port<=15;port++) {
            int proxy=port;
            Output<Double> tempOutput = addOutput("Port "+port+" Current", () -> panel.getCurrent(proxy));
            currents[proxy]=tempOutput;
        }
    }
    
    private Runnable checkCurrent = new Runnable() {
        @Override
        public void run() {
            for (int i=0;i<currents.length;i++) {
                if (currents[i].get() > currentLimit[i]) {
                    theLogger.log("WARN", "Excess current of "+currents[i].get()+" through PDP Port "+i+"!");
                }
            }
        }
    };
    
    @Override
    protected void begin() {
        iterTimer.start();
    }
    
    @Override
    protected void run() {
        // TODO: Find good time interval
        iterTimer.runEvery(0.5, checkCurrent);
    }
    
    @Override
    protected void end() {
        iterTimer.stopAndReset();
    }

}
