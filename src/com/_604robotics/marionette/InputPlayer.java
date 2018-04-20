package com._604robotics.marionette;

public class InputPlayer {
    private InputRecording playbackRecording;
    private double playbackTimestamp;
    private int playbackFrame;

    public void startPlayback (final InputRecording recording) {
        playbackRecording = recording;
        playbackTimestamp = System.currentTimeMillis();
        playbackFrame = 0;

        System.out.println("MARIONETTE PLAYBACK START [frameCount = " + playbackRecording.getFrameCount() + "]");
    }

    public void stopPlayback () {
        playbackRecording = null;
        System.out.println("MARIONETTE PLAYBACK STOP");
    }

    public boolean isPlaying () {
        return advanceFrame();
    }
    
    private double getRawPlaybackTime () {
        return System.currentTimeMillis() - playbackTimestamp;
    }

    public double getPlaybackTime () {
        return advanceFrame() ? getRawPlaybackTime() : 0;
    }

    public int getAxisCount (final int joystick, final int defaultValue) {
        return advanceFrame() ? playbackRecording.getAxisCount(joystick) : defaultValue;
    }

    public int getButtonCount (final int joystick, final int defaultValue) {
        return advanceFrame() ? playbackRecording.getButtonCount(joystick) : defaultValue;
    }

    public double getRawAxis (final int joystick, final int axis, double defaultValue) {
        if (advanceFrame()) {
            return playbackRecording.getRawAxis(playbackFrame, joystick, axis);
        }
        return defaultValue;
    }

    public boolean getRawButton (final int joystick, final int button, boolean defaultValue) {
        if (advanceFrame()) {
            return playbackRecording.getRawButton(playbackFrame, joystick, button);
        }
        return defaultValue;
    }

    private boolean advanceFrame () {
        if (playbackRecording == null) {
            return false;
        }

        final double elapsedTime = getRawPlaybackTime();
        while (playbackFrame < playbackRecording.getFrameCount() && elapsedTime >= playbackRecording.getTimestamp(playbackFrame + 1)) {
            System.out.println(
                    "* advancing frame [" + playbackRecording.getTimestamp(playbackFrame) + " -> " +
                            playbackRecording.getTimestamp(playbackFrame + 1) + "]");
            ++playbackFrame;
        }

        if (playbackFrame >= playbackRecording.getFrameCount()) {
            System.out.println("(out of frames)");
            stopPlayback();
            return false;
        }
        System.out.println("elapsedTime = " + elapsedTime + "; frameTime = " + playbackRecording.getTimestamp(playbackFrame) + "; frame = " + playbackFrame);

        return true;
    }
}