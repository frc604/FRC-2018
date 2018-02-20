package com._604robotics.marionette;

public class InputPlayer implements InputSource {
    private final InputRecording recording;
    private final double startTimestamp;

    private int frame = 0;

    public InputPlayer (final InputRecording recording) {
        this.recording = recording;
        startTimestamp = System.currentTimeMillis();
    }

    public double getElapsedTime () {
        return System.currentTimeMillis() - startTimestamp;
    }

    @Override
    public int getAxisCount () {
        return recording.getAxisCount();
    }

    @Override
    public int getButtonCount () {
        return recording.getButtonCount();
    }

    private boolean hasRecordingEnded () {
        return frame >= recording.getFrameCount();
    }

    @Override
    public double getRawAxis (final int axis) {
        advanceFrame();
        if (hasRecordingEnded()) {
            return 0;
        }
        return recording.getRawAxis(frame, axis);
    }

    @Override
    public boolean getRawButton (final int button) {
        advanceFrame();
        if (hasRecordingEnded()) {
            return false;
        }
        return recording.getRawButton(frame, button);
    }

    private void advanceFrame () {
        final double elapsedTime = getElapsedTime();
        while (frame < recording.getFrameCount() && elapsedTime >= recording.getTimestamp(frame + 1)) {
            ++frame;
        }
    }
}