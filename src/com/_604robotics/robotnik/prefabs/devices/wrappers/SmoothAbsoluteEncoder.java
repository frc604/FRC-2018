package com._604robotics.robotnik.prefabs.devices.wrappers;

import com._604robotics.robotnik.prefabs.devices.AbsoluteEncoder;

import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;

public class SmoothAbsoluteEncoder implements PIDSource {
    public final AbsoluteEncoder source;
    private int offset = 0;
    private double prevResult;

    public SmoothAbsoluteEncoder(AbsoluteEncoder source) {
        this.source = source;
        prevResult = source.getAngle();
    }

    public void resetOffset() {
        offset = 0;
    }
    
    public void setZero() {
        resetOffset();
        source.setZero();
    }
    
    public void setZero(double angle) {
        resetOffset();
        source.setZeroAngle(angle);
    }

    @Override
    public void setPIDSourceType(PIDSourceType pidSource) {
        if (pidSource != PIDSourceType.kDisplacement) {
            throw new IllegalArgumentException(
                    "SmoothAbsoluteEncoder wrapper class only allows PIDSourceType.kDisplacement");
        }
    }

    @Override
    public PIDSourceType getPIDSourceType() {
        return PIDSourceType.kDisplacement;
    }

    @Override
    public double pidGet() {
        // Choose 90 degree offsets
        // Larger offsets can introduce bugs with opposite directions
        // Nyquist limits also make further compensations pointless
        double currentAngle = source.getAngle();
        if (prevResult > 270 && currentAngle < 90) {
            offset++;
        } else if (prevResult < 90 && currentAngle > 270) {
            offset--;
        }
        prevResult = currentAngle;
        return currentAngle + 360 * offset;
    }

}
