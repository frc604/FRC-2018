package com._604robotics.robotnik;

import com._604robotics.robotnik.prefabs.flow.SmartTimer;
import edu.wpi.first.wpilibj.DriverStation;

import java.io.PrintWriter;
import java.io.StringWriter;

/**
 * Generic logging class that prints to standard output.
 */
public class Logger {
    private static SmartTimer bootTimer = new SmartTimer();

    static {
        bootTimer.start();
    }

    private final String name;

    public Logger (String name) {
        this.name = name;
    }

    public Logger (String parentName, String name) {
        this.name = parentName + ":" + name;
    }

    public Logger (@SuppressWarnings("rawtypes") Class c) {
        this(c.getSimpleName());
    }

    public Logger (@SuppressWarnings("rawtypes") Class c, String name) {
        this(c.getSimpleName(), name);
    }

    public Logger (@SuppressWarnings("rawtypes") Class c, @SuppressWarnings("rawtypes") Class c2) {
        this(c.getSimpleName(), c2.getSimpleName());
    }

    /**
     * Logs a message to the console. <br>
     * The format is as follows: <br>
     * <pre> * [Boot time: $bootTime] [Match Time: $matchTime] <br>- [$level] [$name] $message</pre>
     * @param level the value of $level above
     * @param message the message to log in $message above
     */
    public void log (String level, String message) {
        synchronized(System.out){
            System.out.println(
                    "* [Boot Time: " + bootTimer.get() +
                    "]\t[Match Time: " + DriverStation.getInstance().getMatchTime() +
                    "]\n- [" + level +
                    "]\t[" + name +
                    "]\t" + message +
                    "\n");
        }
    }

    /**
     * Logs a message to the console with a level of "INFO".
     * @param message the message to log
     */
    public void info (String message) {
        log("INFO", message);
    }

    /**
     * Prints the stack trace of an exception and logs a message to the console with a level of "ERROR".
     * @param message the message to log
     * @param t the throwable for which a stack trace should be printed
     */
    public void error (String message, Throwable t) {
        final StringWriter sw = new StringWriter();
        final PrintWriter pw = new PrintWriter(sw);
        pw.println(message);
        t.printStackTrace(pw);

        log("ERROR", sw.toString());
    }
}