package com._604robotics.robotnik;

import com._604robotics.robotnik.prefabs.flow.SmartTimer;

import java.util.function.Consumer;

class IterationTimer {
    private final SmartTimer timer = new SmartTimer();
    private final double reportInterval;
    private long iterationCount;

    public IterationTimer (double reportInterval) {
        this.reportInterval = reportInterval;
    }

    public void start () {
        timer.start();
    }

    public void stop () {
        timer.stopAndReset();
        iterationCount = 0;
    }

    public void sample (Consumer<Double> report) {
        if (reportInterval == 0) {
            return;
        }

        ++iterationCount;

        timer.runEvery(reportInterval, () -> {
            report.accept(timer.get() / (double) iterationCount);
            iterationCount = 0;
        });
    }
}