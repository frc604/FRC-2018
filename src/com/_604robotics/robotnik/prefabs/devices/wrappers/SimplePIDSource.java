package com._604robotics.robotnik.prefabs.devices.wrappers;

import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;

public abstract class SimplePIDSource implements PIDSource {
    private final PIDSourceType type;

    public SimplePIDSource (PIDSourceType type) {
        this.type = type;
    }

    @Override
    public PIDSourceType getPIDSourceType () {
        return type;
    }

    @Override
    public void setPIDSourceType (PIDSourceType pidSource) {
        if (pidSource != type) {
            throw new IllegalArgumentException("PID source only supports " + type);
        }
    }
}
