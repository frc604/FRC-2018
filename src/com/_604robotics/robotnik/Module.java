package com._604robotics.robotnik;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

import java.util.ArrayList;
import java.util.List;

public abstract class Module {
	private final NetworkTableInstance network = NetworkTableInstance.create();
	
    private final NetworkTable table;

    private final NetworkTable inputsTable;

    private final NetworkTable outputsTable;

    private final NetworkTable activeActionTable;
    private final NetworkTable activeActionInputsTable;
    private final NetworkTable activeActionOutputsTable;

    private final String name;

    private Action defaultAction;
    private Action runningAction;
    private Action activeAction;

    @SuppressWarnings("rawtypes")
    private final List<Input> inputs = new ArrayList<>();
    @SuppressWarnings("rawtypes")
    private final List<OutputProxy> outputs = new ArrayList<>();

    private long epoch = 0;

    protected void begin () {}
    protected void run () {}
    protected void end () {}

    /**
     * @param name the name of the module class
     */
    public Module (String name) {
        this.name = name;

        table = network.getTable("robotnik")
                .getSubTable("modules")
                .getSubTable(name);

        inputsTable = table.getSubTable("inputs");
        outputsTable = table.getSubTable("outputs");

        activeActionTable = table.getSubTable("activeAction");
        activeActionTable.getEntry("name").setString("");
        activeActionTable.getEntry("inputList").setString("");

        activeActionInputsTable = activeActionTable.getSubTable("inputs");
        activeActionOutputsTable = activeActionTable.getSubTable("outputs");
    }

    public Module (Class klass) {
        this(klass.getSimpleName());
    }
    
    /**
     * @return the name of the module class that created this class
     */
    public String getName () {
        return name;
    }

    public Action getRunningAction () {
        return runningAction;
    }

    protected void setDefaultAction (Action action) {
        this.defaultAction = action;
    }

    protected <T> Input<T> addInput (String name, T defaultValue) {
        final Input<T> input = new Input<>(this, name, defaultValue);
        inputs.add(input);
        inputsTable.getEntry( input.getName() ).setValue( input );
        return input;
    }

    protected <T> Output<T> addOutput (String name, Output<T> output) {
        final OutputProxy<T> proxy = new OutputProxy<>(name, output);
        outputs.add(proxy);
        outputsTable.getEntry( proxy.getName() ).setValue( proxy );
        return proxy;
    }

    long getEpoch () {
        return epoch;
    }

    void prepare () {
        ++epoch;

        for (@SuppressWarnings("rawtypes") OutputProxy output : outputs) {
            Reliability.swallowThrowables(output::update,
                    "Error updating output " + output.getName() + " of module " + getName());
            // Error here due to output.get coming from enum
            // Must be one of Boolean Number String byte[] boolean[] double[] Boolean[] Number[] String[]
            //System.out.println("Put key:"+output.getName()+" value:"+output.get());
            if (output.get()==null) {
            	outputsTable.getEntry( output.getName() ).setString( "null" );
            } else {
            	outputsTable.getEntry( output.getName() ).setString( output.get().toString() ); // Should this be toString(), or something else?ns
            }
        }

        if (runningAction != null) {
            runningAction.updateOutputs(activeActionOutputsTable);
        }

        activeAction = defaultAction;
    }

    void activate (Action action) {
        activeAction = action;
    }

    void update () {
        for (@SuppressWarnings("rawtypes") Input input : inputs) {
        	inputsTable.getEntry( input.getName() ).setValue( input.get() );
        }
    }

    void execute () {
        if (activeAction != runningAction) {
            if (runningAction != null) {
                Reliability.swallowThrowables(runningAction::terminate,
                        "Error in end() of action " + runningAction.getName() + " of module " + getName());
            }

            runningAction = activeAction;

            if (activeAction == null) {
                activeActionTable.getEntry("name").setString("");
                activeActionTable.getEntry("inputList").setString("");
            } else {
                activeAction.updateActiveAction(activeActionTable);
                Reliability.swallowThrowables(activeAction::initiate,
                        "Error in begin() of action " + activeAction.getName() + " of module " + getName());
            }
        }

        if (activeAction != null) {
            activeAction.updateInputs(activeActionInputsTable);
            Reliability.swallowThrowables(activeAction::run,
                    "Error in run() of action " + activeAction.getName() + " of module " + getName());
        }
    }

    void terminate () {
        if (runningAction != null) {
            Reliability.swallowThrowables(runningAction::terminate,
                    "Error in end() of action " + runningAction.getName() + " of module " + getName());
        }
        runningAction = null;

        activeActionTable.getEntry("name").setString("");
        activeActionTable.getEntry("inputList").setString("");
    }
}