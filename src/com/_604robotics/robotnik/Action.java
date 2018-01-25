package com._604robotics.robotnik;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.tables.ITable;

public abstract class Action {
    private final Module parent;
    private final String name;

    @SuppressWarnings("rawtypes")
    private final List<Input> inputs = new ArrayList<>();
    private String inputListValue = "";

    @SuppressWarnings("rawtypes")
    private final List<OutputProxy> outputs = new ArrayList<>();
    private String outputListValue = "";

    private boolean running = false;

    public Action (Module parent, String name) {
        this.parent = parent;
        this.name = name;
    }

    public Action (Module parent, Class klass) {
        this(parent, klass.getSimpleName());
    }

    public String getName () {
        return name;
    }

    public boolean isRunning () {
        return running;
    }

    protected <T> Input<T> addInput (String name, T defaultValue) {
        // Only one synchronized method call: no need to synchronize this
        return addInput(name, defaultValue, false);
    }
    synchronized protected <T> Input<T> addInput (String name, T defaultValue, boolean persistent) {
        if (name.isEmpty()) {
            throw new IllegalArgumentException("Input names may not be empty");
        }
        if (name.contains(",")) {
            throw new IllegalArgumentException("Input names may not contain commas");
        }

        final Input<T> input = new Input<>(parent, name, defaultValue, persistent);
        inputs.add(input);
        inputListValue = inputListValue.isEmpty() ? input.getName() : inputListValue + "," + input.getName();
        return input;
    }

    synchronized protected <T> Output<T> addOutput (String name, Output<T> output) {
        if (name.isEmpty()) {
            throw new IllegalArgumentException("Output names may not be empty");
        }
        if (name.contains(",")) {
            throw new IllegalArgumentException("Output names may not contain commas");
        }

        final OutputProxy<T> proxy = new OutputProxy<>(name, output);
        outputs.add(proxy);
        outputListValue = outputListValue.isEmpty() ? name : outputListValue + "," + name;
        return proxy;
    }

    public void activate () {
        parent.activate(this);
    }

    @Deprecated
    void updateActiveAction (ITable activeActionTable) {
        activeActionTable.putString("name", getName());
        activeActionTable.putString("inputList", inputListValue);
        activeActionTable.putString("outputList", outputListValue);
    }
    
    void updateActiveAction( NetworkTable activeActionTable ) {
    	activeActionTable.getEntry("name").setString( getName() );
    	activeActionTable.getEntry("inputList").setString(inputListValue);
    	activeActionTable.getEntry("outputList").setString(outputListValue);
    }

    @Deprecated
    synchronized void updateInputs (ITable activeActionInputsTable) {
        for (@SuppressWarnings("rawtypes") Input input : inputs) {
            Object res=input.get();
            if (res==null) {
                System.err.println("Got null input.get!");
                System.err.println("Input name is "+input.getName());
            } else {
                activeActionInputsTable.putValue(input.getName(), res);
            }
        }
    }
    
    synchronized void updateInputs( NetworkTable activeActionInputsTable ) {
    	for( @SuppressWarnings("rawtypes") Input input : inputs ) {
    		Object res = input.get();
    		if( res == null ) {
    			System.err.println("Got null input.get!");
    			System.err.println("Input name is " + input.getName() );
    		} else {
    			activeActionInputsTable.getEntry( input.getName() ).setValue(res);
    		}
    	}
    }

    @Deprecated
    synchronized void updateOutputs (ITable activeActionOutputsTable) {
        for (@SuppressWarnings("rawtypes") OutputProxy output : outputs) {
            Reliability.swallowThrowables(output::update,
                    "Error updating output " + output.getName() + " of action " + getName());
            activeActionOutputsTable.putValue(output.getName(), output.get());
        }
    }
    
    synchronized void updateOutputs( NetworkTable acitveActionOutputsTable ) {
    	for( @SuppressWarnings("rawtypes") OutputProxy output : outputs ) {
    		Reliability.swallowThrowables( output::update,
    				"Error updating output " + output.getName() + " of action " + getName() );
    		acitveActionOutputsTable.getEntry( output.getName() ).setValue( output.get() );
    	}
    }

    synchronized void initiate () {
        running = true;
        begin();
    }

    void terminate () {
        running = false;
        end();
    }

    protected void begin () {}
    protected void run () {}
    protected void end () {}
}