package com._604robotics.marionette;

import com._604robotics.robot2018.constants.Calibration;
import com._604robotics.robotnik.prefabs.flow.SmartTimer;

public class InputPlayer {
    private InputRecording playbackRecording;
    private double playbackTimestamp;
    private int playbackFrame;
    private SmartTimer timer;
    private double timerSetpoint;

    public void startPlayback (final InputRecording recording) {
        playbackRecording = recording;
        playbackTimestamp = System.currentTimeMillis();
        playbackFrame = 0;
        timer = new SmartTimer();
        timerSetpoint = 0;
        timer.start();
    }

    public void stopPlayback () {
        playbackRecording = null;
        timer.stop();
        timer.reset();
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
        while (playbackFrame < playbackRecording.getFrameCount() && timer.get() >= timerSetpoint + Calibration.PLAYBACK_DELAY) {
            ++playbackFrame;
            timerSetpoint = timer.get();
        }

        if (playbackFrame >= playbackRecording.getFrameCount()) {
            stopPlayback();
            return false;
        }

        return true;
    }
}