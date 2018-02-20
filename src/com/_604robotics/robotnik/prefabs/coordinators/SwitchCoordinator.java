package com._604robotics.robotnik.prefabs.coordinators;

import java.beans.Expression;
import java.util.HashMap;

import com._604robotics.robotnik.Coordinator;
import com._604robotics.robotnik.Logger;

public class SwitchCoordinator extends Coordinator {
	private final Logger logger;
    
	private GameDataCoordinator gdc;
	
	private HashMap<String, StatefulCoordinator> cases;
	
    public SwitchCoordinator (String name) {
    	gdc = new GameDataCoordinator();
    	cases = new HashMap<String, StatefulCoordinator>();
    	logger = new Logger(SwitchCoordinator.class, name);
    }
    public SwitchCoordinator (Class klass) {
    	this(klass.getSimpleName());
    	gdc = new GameDataCoordinator();
    	cases = new HashMap<String, StatefulCoordinator>();
    }
    
    @Override
    public void begin() {
    	logger.info("Begin");
    	
    }
}
