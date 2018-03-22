package com._604robotics.robotnik.prefabs.coordinators;

import java.util.HashMap;
import java.util.concurrent.ExecutionException;

import com._604robotics.robotnik.Coordinator;
import com._604robotics.robotnik.Logger;

// TODO: use regex
public class SwitchCoordinator extends Coordinator {
	private final Logger logger;
	private AsynchronousGameDataRetriever gdc;
	private HashMap<String, Coordinator> cases;
	private String gameData;
	private boolean gotData;
	private boolean endedGameDataCoordinator;
	private boolean started;
	private Coordinator active;
	private Coordinator defaultCoordinator;
	
    public SwitchCoordinator (String name, AsynchronousGameDataRetriever ret) {
    	logger = new Logger(SwitchCoordinator.class, name);
    	cases = new HashMap<String, Coordinator>();
    	gotData = false;
    	gameData = "";
    	started = false;
    	gdc = ret;
    }
    public SwitchCoordinator (Class<?> klass, AsynchronousGameDataRetriever ret) {
    	this(klass.getSimpleName(), ret);
    }
    
    public void addCase( String[] conditions, Coordinator coordinator ) {
    	for( String condition : conditions ) {
    		cases.put(condition, coordinator);
    	}
    }
    public void addDefault(Coordinator coordinator) {
        defaultCoordinator = coordinator;
    }
    
    @Override
    public void begin() {
    	logger.info("Begin");
    	gameData = "";
    	gotData = false;
    	endedGameDataCoordinator = false;
    	started = false;
    }
    
    @Override
    public boolean run() {
        if (!gdc.isReady()) {
            return true;
        } else if (!started) {
            try {
                gameData = gdc.get();
            } catch (InterruptedException e) {
                // Should never happen
                gameData = "";
                e.printStackTrace();
            } catch (ExecutionException e) {
                // Should never happen
                gameData = "";
                e.printStackTrace();
            }
    	    active = cases.getOrDefault(gameData, defaultCoordinator);
    		active.start();
    		started = true;
    	} else {
    		return active.execute();
    	}
    	return true;
    }
    
    @Override
    public void end() {
    	active.stop();
        logger.info("End");
    }
}
