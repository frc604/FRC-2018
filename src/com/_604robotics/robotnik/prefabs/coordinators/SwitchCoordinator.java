package com._604robotics.robotnik.prefabs.coordinators;

import java.util.HashMap;

import com._604robotics.robotnik.Coordinator;
import com._604robotics.robotnik.Logger;

// TODO: use regex
public class SwitchCoordinator extends Coordinator {
	private final Logger logger;
	private GameDataCoordinator gdc;
	private HashMap<String, Coordinator> cases;
	private String gameData;
	private boolean gotData;
	private boolean endedGameDataCoordinator;
	private boolean started;
	private Coordinator active;
	
    public SwitchCoordinator (String name) {
    	logger = new Logger(SwitchCoordinator.class, name);
    	gdc = new GameDataCoordinator();
    	cases = new HashMap<String, Coordinator>();
    	gameData = "";
    	endedGameDataCoordinator = false;
    	started = false;
    }
    public SwitchCoordinator (Class klass) {
    	this(klass.getSimpleName());
    }
    
    public void addCase( String[] conditions, Coordinator coordinator ) {
    	for( String condition : conditions ) {
    		cases.put(condition, coordinator);
    	}
    }
    
    @Override
    public void begin() {
    	logger.info("Begin");
    	gdc.start();
    	gameData = "";
    	endedGameDataCoordinator = false;
    	started = false;
    }
    
    @Override
    public boolean run() {
    	if( !gotData ) {
    		if( !gdc.execute() ) {
        		gameData = gdc.getGameData();
        		gotData = true;
        	}
    	} else if( !endedGameDataCoordinator ) {
    		gdc.stop();
    		endedGameDataCoordinator = true;
    	} else if( !started ) {
    		active = cases.get(gameData);
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
