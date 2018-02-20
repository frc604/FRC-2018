package com._604robotics.marionette;

public class JoystickDescriptor {
    private final int axisCount;
    private final int buttonCount;

    public JoystickDescriptor (final int axisCount, final int buttonCount) {
        this.axisCount = axisCount;
        this.buttonCount = buttonCount;
    }

    public int getAxisCount () {
        return axisCount;
    }

    public int getButtonCount () {
        return buttonCount;
    }
}