package com._604robotics.robotnik;

import edu.wpi.first.networktables.NetworkTable;

@Deprecated
class TableIndex { // Stores multiple values under one key. Why, though.
    private final NetworkTable table;
    private final String key;

    public TableIndex (NetworkTable table, String key) {
        this.table = table;
        this.key = key;
        
        table.getEntry(key).setString("");
    }

    public void add( String type, String name ) {
        if (name.isEmpty()) {
            throw new IllegalArgumentException(type + " names may not be empty");
        }
        if (name.contains(",")) {
            throw new IllegalArgumentException(type + " names may not contain commas");
        }
        final String oldList = table.getEntry(key).getString("");
        
        table.getEntry(key).setString( oldList.isEmpty() ? name : oldList + "," + name );
    }
}