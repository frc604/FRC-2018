package com._604robotics.robotnik;

/**
 * A template class that represents an input.
 * 
 * @param <T> The type of input
 */
public class Input<T> {

    private final Module parent;

    private final String name;

    private final T defaultValue;
    private T value;
    private T prevValue;
    private long valueEpoch = -1;
    private final boolean persistent;

    Input (Module parent, String name, T defaultValue) {
        this(parent, name, defaultValue, false);
    }
    Input (Module parent, String name, T defaultValue, boolean persistent) {
        this.parent = parent;
        this.name = name;
        this.defaultValue = defaultValue;
        this.prevValue = defaultValue;
        this.persistent = persistent;
    }

    public String getName () {
        return name;
    }

    public boolean isPersistent() {
        return persistent;
    }

    public T get () {
        if (valueEpoch == parent.getEpoch()) {
            return value;
        } else {
            if (persistent) {
                return prevValue;
            } else {
                return defaultValue;
            }
        }
    }

    public synchronized void set (T value) {
        this.prevValue = this.value;
        this.value = value;
        valueEpoch = parent.getEpoch();
    }

    public boolean isFresh () {
        final long currentEpoch = parent.getEpoch();
        return valueEpoch == currentEpoch || valueEpoch == currentEpoch - 1;
    }
}