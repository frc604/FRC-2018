package com._604robotics.robotnik;

class Reliability {
    private static Logger logger = new Logger(Reliability.class);

    private Reliability () {}

    static void swallowThrowables (Runnable f, String errorPrelude) {
        try {
            f.run();
        } catch (Throwable t) {
            logger.error(errorPrelude, t);
        }
    }
}