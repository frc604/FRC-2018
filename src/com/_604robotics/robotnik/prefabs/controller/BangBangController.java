package com._604robotics.robotnik.prefabs.controller;

import java.util.Timer;
import java.util.TimerTask;

import com._604robotics.robotnik.utils.annotations.Untested;

import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;

//UNTESTED
//             _            _           _
// _   _ _ __ | |_ ___  ___| |_ ___  __| |
//| | | | '_ \| __/ _ \/ __| __/ _ \/ _` |
//| |_| | | | | ||  __/\__ \ ||  __/ (_| |
// \__,_|_| |_|\__\___||___/\__\___|\__,_|
/**
 * <p>
 * Implements a bang-bang type speed controller.
 * </p>
 * Due to the nature of the algorithm, precise update frequencies are
 * unimportant.
 * This implementation uses PIDSource and PIDOutput interfaces for
 * compatibility.
 */
@Deprecated @Untested("Needs to be tested with actual spinning wheel")
public class BangBangController {
    private double threshold;
    private final double power;
    private final PIDOutput output;
    private final PIDSource source;
    private boolean enabled;
    private TimerTask task;
    // This is the java.util.Timer on purpose
    // Structure copied from PIDController
    private Timer timer;

    private class BangBangTask extends TimerTask {

        private BangBangController controller;

        public BangBangTask(BangBangController control) {
            if (control == null) {
                throw new IllegalArgumentException("BangBangController passed in is null");
            }
            this.controller = control;
        }

        @Override
        public void run() {
            if (enabled) {
                double inputValue;
                double threshold;
                synchronized (controller) {
                    inputValue = controller.source.pidGet();
                    threshold = controller.getThreshold();
                }
                if (Math.abs(inputValue) > Math.abs(threshold)) {
                    controller.output.pidWrite(0);
                } else {
                    controller.output.pidWrite(controller.power);
                }
            }
        }

    }

    public BangBangController(double threshold, double power, PIDSource source, PIDOutput output) {
        if (source.getPIDSourceType() != PIDSourceType.kRate) {
            throw new IllegalArgumentException("BangBang controller requires kRate!");
        }
        this.source = source;
        this.output = output;
        this.power = power;
        task = new BangBangTask(this);
        timer = new Timer();
        timer.schedule(task, 0, 50);
    }

    public void free() {
        task.cancel();
        task = null;
    }

    public synchronized void enable() {
        enabled = true;
    }

    public synchronized void disable() {
        output.pidWrite(0);
        enabled = false;
    }

    public boolean isEnabled() {
        return enabled;
    }

    public double getThreshold() {
        return threshold;
    }

    public synchronized void setThreshold(double threshold) {
        this.threshold = threshold;
    }

}
