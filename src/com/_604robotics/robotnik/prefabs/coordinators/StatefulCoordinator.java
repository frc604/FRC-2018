package com._604robotics.robotnik.prefabs.coordinators;

import com._604robotics.robotnik.Coordinator;
import com._604robotics.robotnik.Logger;
import com._604robotics.robotnik.utils.Pair;

import java.util.ArrayList;
import java.util.List;

public abstract class StatefulCoordinator extends Coordinator {
    private final Logger logger;

    private List<Pair<String, Coordinator>> states = new ArrayList<>();
    private int stateIndex = 0;

    public StatefulCoordinator (String name) {
        logger = new Logger(StatefulCoordinator.class, name);
    }

    public StatefulCoordinator (Class<?> klass) {
        this(klass.getSimpleName());
    }

    public void addState (String description, Coordinator coordinator) {
        states.add(new Pair<>(description, coordinator));
    }

    @Override
    public void begin () {
        logger.info("Begin");
        stateIndex = 0;
        enterState();
    }

    @Override
    public boolean run () {
        final Pair<String, Coordinator> currentState = getCurrentState();
        if (currentState == null) {
            return false;
        }

        if (!currentState.getValue().execute()) {
            exitState();
            ++stateIndex;
            return enterState();
        }
        return true;
    }

    @Override
    public void end () {
        exitState();
        logger.info("End");
    }

    private boolean enterState () {
        final Pair<String, Coordinator> currentState = getCurrentState();
        if (currentState == null) {
            logger.info("Null State");
            return false;
        }

        logger.info("Enter State: " + currentState.getKey());
        currentState.getValue().start();
        return true;
    }

    private void exitState () {
        final Pair<String, Coordinator> currentState = getCurrentState();
        if (currentState != null) {
            currentState.getValue().stop();
            logger.info("Exit State: " + currentState.getKey());
        }
    }

    private Pair<String, Coordinator> getCurrentState () {
        if (stateIndex >= states.size()) {
            return null;
        }
        return states.get(stateIndex);
    }
}