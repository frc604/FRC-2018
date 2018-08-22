package com._604robotics.robotnik.prefabs.coordinators;

import com._604robotics.robotnik.Coordinator;
import com._604robotics.robotnik.Logger;
import com._604robotics.robotnik.utils.annotations.Untested;

import java.util.HashMap;
import java.util.Map;

@Untested("Does not guarantee the start or finish time of any Coordinator")
public class SimultaneousCoordinator extends Coordinator {
    private final Logger logger;
    private Map<Coordinator, Boolean[]> allTasks;

    public SimultaneousCoordinator( Coordinator[] tasks ) {
        this.logger = new Logger( SimultaneousCoordinator.class);
        this.allTasks = new HashMap<>();
        for( Coordinator c : tasks ) {
            this.allTasks.put( c, new Boolean[]{false, false} );
        }
    }

    @Override
    public void begin() {
        logger.info("Begin");
        for( Coordinator c : allTasks.keySet() ) {
            logger.info("Starting " + c.toString());
            c.start();
        }
    }

    @Override
    public boolean run() {
        boolean running = false;

        for( Coordinator c : allTasks.keySet() ) {
            if( c.execute() ) {
                running = true;
            }
        }

        return running;
    }

    @Override
    public void end() {
        for( Coordinator c : allTasks.keySet() ) {
            c.stop();
            logger.info("Stopped " + c.toString());
        }
        logger.info("End");
    }

    public void addCoordinator( Coordinator c ) {
        allTasks.put( c, new Boolean[]{false,false} );
    }

}
