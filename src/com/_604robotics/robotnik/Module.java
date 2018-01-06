package com._604robotics.robotnik;

import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.tables.ITable;

import java.util.ArrayList;
import java.util.List;

public abstract class Module {
    private final ITable table;

    private final ITable inputsTable;
    private final TableIndex inputsTableIndex;

    private final ITable outputsTable;
    private final TableIndex outputsTableIndex;

    private final ITable activeActionTable;
    private final ITable activeActionInputsTable;
    private final ITable activeActionOutputsTable;

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

    public Module (String name) {
        this.name = name;

        table = NetworkTable.getTable("robotnik")
                .getSubTable("modules")
                .getSubTable(name);

        inputsTable = table.getSubTable("inputs");
        inputsTableIndex = new TableIndex(table, "inputList");

        outputsTable = table.getSubTable("outputs");
        outputsTableIndex = new TableIndex(table, "outputList");

        activeActionTable = table.getSubTable("activeAction");
        activeActionTable.putString("name", "");
        activeActionTable.putString("inputList", "");

        activeActionInputsTable = activeActionTable.getSubTable("inputs");
        activeActionOutputsTable = activeActionTable.getSubTable("outputs");
    }

    public Module (Class klass) {
        this(klass.getSimpleName());
    }

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
        inputsTableIndex.add("Input", input.getName());
        return input;
    }

    protected <T> Output<T> addOutput (String name, Output<T> output) {
        final OutputProxy<T> proxy = new OutputProxy<>(name, output);
        outputs.add(proxy);
        outputsTableIndex.add("Output", name);
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
                outputsTable.putString(output.getName(), "null");
            } else {
                outputsTable.putString(output.getName(), output.get().toString());
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
            inputsTable.putValue(input.getName(), input.get());
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
                activeActionTable.putString("name", "");
                activeActionTable.putString("inputList", "");
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

        activeActionTable.putString("name", "");
        activeActionTable.putString("inputList", "");
    }
}