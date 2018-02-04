package com._604robotics.robotnik.prefabs.coordinators;

import com._604robotics.robotnik.Coordinator;
import com._604robotics.robotnik.utils.annotations.Untested;

@Deprecated @Untested("Needs to be actually used")
public class SimultaneousCoordinator extends Coordinator {
    
    private Coordinator coordinators[];

    public SimultaneousCoordinator(Coordinator... coords) {
        coordinators=coords;
    }

    @Override
    protected void begin() {
        for (Coordinator co:coordinators) {
            co.start();
        }
    }

    @Override
    protected boolean run() {
        boolean currentstate=false;
        for (Coordinator co:coordinators) {
            // execute does nothing if the coordinator is in stopped state
            currentstate = currentstate || co.execute();
        }
        return currentstate;
    }

    @Override
    protected void end() {
        for (Coordinator co:coordinators) {
            co.stop();
        }
    }

}
