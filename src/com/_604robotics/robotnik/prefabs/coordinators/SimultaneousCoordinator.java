package com._604robotics.robotnik.prefabs.coordinators;

import java.util.ArrayList;

import com._604robotics.robotnik.Coordinator;
import com._604robotics.robotnik.utils.annotations.Untested;

@Deprecated @Untested("Needs to be actually used")
public class SimultaneousCoordinator extends Coordinator {
    
    private ArrayList<Coordinator> coordinators = new ArrayList<>();

    public SimultaneousCoordinator(Coordinator... coords) {
        for (Coordinator co: coords) {
            addCoordinator(co);
        }
    }
    
    public void addCoordinator(Coordinator coord) {
        coordinators.add(coord);
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
        // Continue running until all are done
        for (Coordinator co:coordinators) {
            System.out.println("Running co "+co.toString());
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
