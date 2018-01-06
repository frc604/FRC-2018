package com._604robotics.robotnik.prefabs.devices.wrappers;

import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;

/**
 * A PIDSource wrapper that takes a PIDSource and acts as the negative of the PIDSource.
 */
public class InvertPIDSource implements PIDSource {

    private final PIDSource wrappedSource;

    public InvertPIDSource(PIDSource Source) {
        this.wrappedSource = Source;
    }

    @Override
    public void setPIDSourceType(PIDSourceType pidSource) {
        wrappedSource.setPIDSourceType(pidSource);

    }

    @Override
    public PIDSourceType getPIDSourceType() {
        // TODO Auto-generated method stub
        return wrappedSource.getPIDSourceType();
    }

    @Override
    public double pidGet() {
        // TODO Auto-generated method stub
        return -wrappedSource.pidGet();
    }

}
