package com._604robotics.robotnik.prefabs.modules;

import com._604robotics.robotnik.Input;
import com._604robotics.robotnik.Module;
import com._604robotics.robotnik.Output;
import com._604robotics.robotnik.utils.Pair;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList;
import java.util.EnumSet;
import java.util.List;

public class DashboardModule extends Module {
    private enum InputType {
        BOOLEAN, DOUBLE, INTEGER, STRING, ENUM
    }

    private final List<Pair<Input, InputType>> inputs = new ArrayList<>();

    protected Input<Boolean> addDashboardInput (String name, boolean initialValue) {
        SmartDashboard.putBoolean(name, initialValue);
        final Input<Boolean> input = addInput(name, initialValue);
        inputs.add(new Pair<>(input, InputType.BOOLEAN));
        return input;
    }

    protected Input<Double> addDashboardInput (String name, double initialValue) {
        SmartDashboard.putNumber(name, initialValue);
        final Input<Double> input = addInput(name, initialValue);
        inputs.add(new Pair<>(input, InputType.DOUBLE));
        return input;
    }
    
    protected Input<Integer> addDashboardInput (String name, int initialValue) {
        SmartDashboard.putNumber(name, initialValue);
        final Input<Integer> input = addInput(name, initialValue);
        inputs.add(new Pair<>(input, InputType.INTEGER));
        return input;
    }

    protected Input<String> addDashboardInput (String name, String initialValue) {
        SmartDashboard.putString(name, initialValue);
        final Input<String> input = addInput(name, initialValue);
        inputs.add(new Pair<>(input, InputType.STRING));
        return input;
    }

    protected <E extends Enum<E>> Input<E> addDashboardInput (String name, E initialValue) {
        SmartDashboard.putString(name, initialValue.toString());
        final Input<E> input = addInput(name, initialValue);
        inputs.add(new Pair<>(input, InputType.ENUM));
        return input;
    }

    protected Output<Boolean> addDashboardOutput (String name, boolean defaultValue) {
        SmartDashboard.putBoolean(name, defaultValue);
        return addOutput(name, defaultValue, () -> SmartDashboard.getBoolean(name, defaultValue));
    }

    protected Output<Double> addDashboardOutput (String name, double defaultValue) {
        SmartDashboard.putNumber(name, defaultValue);
        return addOutput(name, defaultValue, () -> SmartDashboard.getNumber(name, defaultValue));
    }

    protected Output<String> addDashboardOutput (String name, String defaultValue) {
        SmartDashboard.putString(name, defaultValue);
        return addOutput(name, defaultValue, () -> SmartDashboard.getString(name, defaultValue));
    }

    protected <E extends Enum<E>> Output<E> addDashboardOutput (String name, E defaultValue, Class<E> klass) {
        final SendableChooser<E> chooser = new SendableChooser<>();
        for (E option : EnumSet.allOf(klass)) {
            if (option == defaultValue) {
                chooser.addDefault(option.toString(), option);
            } else {
                chooser.addObject(option.toString(), option);
            }
        }
        SmartDashboard.putData(name, chooser);
        return addOutput(name, defaultValue, chooser::getSelected);
    }

    public DashboardModule (String name) {
        super(name);
    }

    public DashboardModule (Class<?> klass) {
        super(klass);
    }

    @Override
    protected void run () {
        for (Pair<Input, InputType> input : inputs) {
            if (input.getKey().isFresh()) {
                switch (input.getValue()) {
                    case BOOLEAN:
                        SmartDashboard.putBoolean(input.getKey().getName(), (Boolean) input.getKey().get());
                        break;
                    case DOUBLE:
                        SmartDashboard.putNumber(input.getKey().getName(), (Double) input.getKey().get());
                        break;
                    case INTEGER:
                        SmartDashboard.putNumber(input.getKey().getName(), ((Integer) input.getKey().get()).doubleValue());
                        break;
                    case STRING:
                        SmartDashboard.putString(input.getKey().getName(), (String) input.getKey().get());
                        break;
                    case ENUM:
                        SmartDashboard.putString(input.getKey().getName(), input.getKey().get().toString());
                        break;
                }
            }
        }
    }
}