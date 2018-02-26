package com._604robotics.robotnik;

public abstract class Coordinator {
    private boolean running;

    public boolean isRunning () {
        return running;
    }

    public void start () {
        if (!running) {
            running = true;
            begin();
        }
    }

    public boolean execute () {
        if (running) {
            running = run();
            if (!running) {
                end();
            }
        }
        return running;
    }

    public void stop () {
        if (running) {
            running = false;
            end();
        }
    }

    protected void begin () {}
    /**
     * Returns FALSE when FINISHED. FALSE. FINISHED. BOTH START WITH 'F'.
     * @return boolean representing whether this Coordinator is running. IS RUNNING.
     */
    protected boolean run () { return true; }
    protected void end () {}
}