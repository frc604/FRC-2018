package com._604robotics.robotnik.prefabs.coordinators;

import com._604robotics.robotnik.Action;
import com._604robotics.robotnik.Coordinator;

import java.util.function.Supplier;

public class ActionCoordinator extends Coordinator {
    private final Action action;
    private final Supplier<Boolean> continueFunc;

    public ActionCoordinator (Action action) {
        this(action, () -> true);
    }

    public ActionCoordinator (Action action, Supplier<Boolean> continueFunc) {
        this.action = action;
        this.continueFunc = continueFunc;
    }

    @Override
    protected boolean run () {
        action.activate();
        return continueFunc.get();
    }
}
