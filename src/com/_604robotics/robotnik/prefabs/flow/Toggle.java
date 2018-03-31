package com._604robotics.robotnik.prefabs.flow;

/**
 * A utility class that provides toggle functionality
 */
public class Toggle {
    private final Pulse pulse = new Pulse();
    private boolean state;

    public Toggle () {
        this(false);
    }

    public Toggle (boolean startInOnState) {
        state = startInOnState;
    }

    public boolean isInOnState () {
        return state;
    }

    public boolean isInOffState () {
        return !state;
    }

    public boolean isEnteringOnState () {
        return state && pulse.isRisingEdge();
    }

    public boolean isEnteringOffState () {
        return !state && pulse.isRisingEdge();
    }

    public boolean isLeavingOnState () {
        return state && pulse.isFallingEdge();
    }

    public boolean isLeavingOffState () {
        return !state && pulse.isFallingEdge();
    }

    public void update (boolean input) {
        pulse.update(input);
        if (pulse.isRisingEdge()) {
            state = !state;
        }
    }

    public void reset (boolean resetToOnState) {
        pulse.reset();
        state = resetToOnState;
    }
}