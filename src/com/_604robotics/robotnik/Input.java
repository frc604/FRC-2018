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

    Input (Module parent, String name, T defaultValue) {
        this.parent = parent;
        this.name = name;
        this.defaultValue = defaultValue;
        this.prevValue = defaultValue;
    }

    public String getName () {
        return name;
    }

    public T get () {
        return valueEpoch == parent.getEpoch()
                ? value
                : defaultValue;
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