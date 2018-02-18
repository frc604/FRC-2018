package com._604robotics.robotnik.prefabs.flow;

import edu.wpi.first.wpilibj.Timer;

import java.util.function.Supplier;

/**
 * A class that extends the built-in WPILib timer.
 */
public class SmartTimer extends Timer {
    private boolean running;

    public boolean isRunning () {
        return running;
    }

    /**
     * Method that starts the timer if it is not already running.
     */
    public void startIfNotRunning () {
        if (!isRunning()) {
            start();
        }
    }

    /**
     * Method that stops and resets the timer in one function call.
     */
    public void stopAndReset () {
        stop();
        reset();
    }
    
    /**
     * Method that restarts the timer in one function call.
     */
    public void restart () {
        stop();
        reset();
        start();
    }

    /**
     * Method that determines whether the timer has passed a certain time.
     * @param time the desired time to reach
     * @return whether the given time has elapsed or not.
     */
    public boolean hasReachedTime (double time) {
        return get() >= time;
    }

    /**
     * Method that runs a runnable until a certain time has elapsed.
     * @param time the time to run the Runnable for
     * @param runnable the Runnable to run
     * @return boolean representing whether the timer has not finished running. Returns FALSE if FINISHED.
     */
    public boolean runUntil (double time, Runnable runnable) {
        if (!hasReachedTime(time)) {
            runnable.run();
            return true;
        }
        return false;
    }

    public <T> T runUntil (double time, T defaultValue, Supplier<T> supplier) {
        if (!hasReachedTime(time)) {
            return supplier.get();
        }
        return defaultValue;
    }

    public <T> T runUntil (double time, Supplier<T> supplier) {
        return runUntil(time, null, supplier);
    }

    /**
     * Method that runs a runnable until after certain time has elapsed.
     * @param time the time after which to run the Runnable
     * @param runnable the Runnable to run
     * @return whether the Runnable was executed or not
     */
    public boolean runAfter (double time, Runnable runnable) {
        if (hasReachedTime(time)) {
            runnable.run();
            return true;
        }
        return false;
    }

    public <T> T runAfter (double time, T defaultValue, Supplier<T> supplier) {
        if (hasReachedTime(time)) {
            return supplier.get();
        }
        return defaultValue;
    }

    public <T> T runAfter (double time, Supplier<T> supplier) {
        return runAfter(time, null, supplier);
    }

    /**
     * Method that runs a Runnable periodically.
     * <b>Note that the timer is reset when the Runnable is executed.</b>
     * @param time the delay between executions of the Runnable
     * @param runnable the Runnable to run periodically
     * @return whether the Runnable was executed
     */
    public boolean runEvery (double time, Runnable runnable) {
        if (hasReachedTime(time)) {
            runnable.run();
            reset();
            return true;
        }
        return false;
    }

    public <T> T runEvery (double time, T defaultValue, Supplier<T> supplier) {
        if (hasReachedTime(time)) {
            final T value = supplier.get();
            reset();
            return value;
        }
        return defaultValue;
    }

    public <T> T runEvery (double time, Supplier<T> supplier) {
        return runEvery(time, null, supplier);
    }

    @Override
    public void start () {
        super.start();
        running = true;
    }

    @Override
    public void stop () {
        super.stop();
        running = false;
    }
}