package com._604robotics.robotnik;

import edu.wpi.first.wpilibj.tables.ITable;

class TableIndex {
    private final ITable table;
    private final String key;

    public TableIndex (ITable table, String key) {
        this.table = table;
        this.key = key;

        table.putString(key, "");
    }

    public void add (String type, String name) {
        if (name.isEmpty()) {
            throw new IllegalArgumentException(type + " names may not be empty");
        }
        if (name.contains(",")) {
            throw new IllegalArgumentException(type + " names may not contain commas");
        }

        final String oldList = table.getString(key, "");
        table.putString(key, oldList.isEmpty() ? name : oldList + "," + name);
    }
}