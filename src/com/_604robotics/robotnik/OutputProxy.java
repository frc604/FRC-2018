package com._604robotics.robotnik;

class OutputProxy<T> implements Output<T> {
    private final String name;
    private final Output<T> source;
    private T value;

    public OutputProxy (String name, Output<T> source) {
        if (name.contains(",")) {
            throw new IllegalArgumentException("Output names may not contain commas");
        }

        this.name = name;
        this.source = source;
    }

	public String getName () {
        return name;
    }

    @Override
    public synchronized T get () {
        return value;
    }

    synchronized void update () {
        value = source.get();
    }
}