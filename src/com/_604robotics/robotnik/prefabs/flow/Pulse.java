package com._604robotics.robotnik.prefabs.flow;

/**
 * A Pulse class that functions like a boolean
 * 
 * This class also provides methods for detecting rsing and falling edges.
 */
public class Pulse {
    private boolean lastInput = false;

    private boolean risingEdge = false;
    private boolean fallingEdge = false;

    public boolean isHigh () {
        return lastInput;
    }

    public boolean isLow () {
        return !lastInput;
    }

    public boolean isRisingEdge () {
        return risingEdge;
    }

    public boolean isFallingEdge () {
        return fallingEdge;
    }

    public void update (boolean input) {
        risingEdge = !lastInput && input;
        fallingEdge = lastInput && !input;

        lastInput = input;
    }
}