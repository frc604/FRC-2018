package com._604robotics.marionette;

import edu.wpi.first.wpilibj.GenericHID;

import java.io.Closeable;
import java.io.IOException;

import com._604robotics.robot2018.constants.Calibration;

public class InputRecorder implements Closeable {
    private final GenericHID[] sources;
    private final InputRecording recording;
    private final Thread thread;

    public InputRecorder (final int maxFrameCount, final GenericHID... sources) {
        this(maxFrameCount, Calibration.RECORD_DELAY, sources);
    }

    public InputRecorder (final int maxFrameCount, final long framePause, final GenericHID... sources) {
        this.sources = sources;

        final JoystickDescriptor[] joystickDescriptors = new JoystickDescriptor[sources.length];
        for (int i = 0; i < sources.length; ++i) {
            joystickDescriptors[i] = new JoystickDescriptor(sources[i].getAxisCount(), sources[i].getButtonCount());
        }
        this.recording = new InputRecording(maxFrameCount, joystickDescriptors);

        thread = new Thread(() -> {
            final double startTimestamp = System.currentTimeMillis();
            int frame = 0;

            while (!Thread.interrupted()) {
                if (frame >= maxFrameCount) {
                    break;
                }

                recording.setTimestamp(frame, System.currentTimeMillis() - startTimestamp);
                for (int i = 0; i < recording.getJoystickCount(); ++i) {
                    for (int j = 0; j < recording.getAxisCount(i); ++j) {
                        recording.setRawAxis(frame, i, j, sources[i].getRawAxis(j));
                    }
                    for (int j = 1; j <= recording.getButtonCount(i); ++j) {
                        recording.setRawButton(frame, i, j, sources[i].getRawButton(j));
                    }
                }

                ++frame;

                try {
                    Thread.sleep(framePause);
                } catch (InterruptedException e) {
                    break;
                }
            }

            recording.setTimestamp(frame, System.currentTimeMillis() - startTimestamp);
            recording.truncateFrameCount(frame);
        });
        thread.start();
    }

    @Override
    public void close () throws IOException {
        thread.interrupt();
        try {
            thread.join();
        } catch (InterruptedException e) {
            throw new IOException(e);
        }
    }

    public InputRecording getRecording () {
        return recording;
    }
}