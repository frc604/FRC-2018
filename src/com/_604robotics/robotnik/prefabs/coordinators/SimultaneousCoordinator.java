package com._604robotics.robotnik.prefabs.coordinators;

import java.util.ArrayList;

import com._604robotics.robotnik.Coordinator;
import com._604robotics.robotnik.utils.annotations.Untested;

@Deprecated @Untested("Needs to be actually used")
public class SimultaneousCoordinator extends Coordinator {
    
    private ArrayList<Coordinator> coordinators;
    private ArrayList<Boolean> started;
    private ArrayList<Boolean> stopped;
    private int index;
    private boolean finished;
    private boolean fullCycle;

    public SimultaneousCoordinator(Coordinator... coords) {
    	coordinators = new ArrayList<Coordinator>();
    	started = new ArrayList<Boolean>();
    	stopped = new ArrayList<Boolean>();
        for (Coordinator co: coords) {
            addCoordinator(co);
            started.add(false);
            stopped.add(false);
        }
        index = 0;
        finished = false;
        fullCycle = false;
    }
    
    public void addCoordinator(Coordinator coord) {
        coordinators.add(coord);
    }

    @Override
    protected void begin() {
    	for( int f=0; f<started.size(); f++ ) {
    		started.set(f, false);
    		stopped.set(f, false);
    	}
    	index = 0;
    	finished = false;
    	fullCycle = false;
    }

    @Override
    protected boolean run() {
    	boolean finishedStarting = true;
    	for( boolean startCheck : started ) {
    		finishedStarting &= startCheck;
    	}
    	
    	boolean currentState = false;
    	boolean stoppedState = true;
    	
    	if( finished ) {
    		stoppedState = false;
    		currentState = true;
    		for( int f=0; f<stopped.size(); f++ ) {
    			if( !stopped.get(f) ) {
    				stopped.set(f, true);
    				coordinators.get(f).stop();
    				System.out.println("Stopped "+coordinators.get(f).toString());
    				stoppedState = true;
    				break;
    			}
    		}
    	} else if( finishedStarting ) {
    		int modulatedIndex = index%coordinators.size();
    		fullCycle |= coordinators.get(modulatedIndex).execute();
    		System.out.println("Executed " + coordinators.get(modulatedIndex).toString());
    		if( !(modulatedIndex == coordinators.size()-1 && !fullCycle) ) {
    			fullCycle = false;
    			currentState = true;
    		}
    		index++;
    	} else {
    		currentState = true;
    		for( int f=0; f<started.size(); f++ ) {
    			if( !started.get(f) ) {
    				started.set(f, true);
    				coordinators.get(f).start();
    				System.out.println("Started " + coordinators.get(f).toString());
    				break;
    			}
    		}
    	}
    	
    	if( !currentState ) {
    		finished = true;
    	}
    	
    	return stoppedState;
    	
//        boolean currentstate=false;
//        // Continue running until all are done
//        for (Coordinator co:coordinators) {
//            System.out.println("Running co "+co.toString());
//            // execute does nothing if the coordinator is in stopped state
//            currentstate = currentstate || co.execute();
//        }
//        return currentstate;
    }

    @Override
    protected void end() {
        // Do nothing
    }

}
