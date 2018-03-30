package com._604robotics.marionette;

import java.io.*;

public class InputRecording {
    private int frameCount;
    private final JoystickDescriptor[] joystickDescriptors;

    private final int totalAxisCount;
    private final int[] axisOffsets;

    private final double[] timestampBuffer;
    private final double[] axisBuffer;
    private final int[] buttonBuffer;

    public InputRecording (final int frameCount, final JoystickDescriptor... joystickDescriptors) {
        this.frameCount = frameCount;
        this.joystickDescriptors = joystickDescriptors;

        axisOffsets = new int[joystickDescriptors.length];
        int axisAccum = 0;
        for (int i = 0; i < joystickDescriptors.length; ++i) {
            axisOffsets[i] = axisAccum;
            axisAccum += joystickDescriptors[i].getAxisCount();
        }
        totalAxisCount = axisAccum;

        timestampBuffer = new double[frameCount + 1];
        axisBuffer = new double[frameCount * totalAxisCount];
        buttonBuffer = new int[frameCount * joystickDescriptors.length];
    }

    public int getFrameCount () {
        return frameCount;
    }

    public void truncateFrameCount (final int newFrameCount) {
        if (newFrameCount > frameCount) {
            throw new IllegalArgumentException(
                    "newFrameCount > frameCount (" + newFrameCount + " > " + frameCount + ")");
        }

        frameCount = newFrameCount;
    }

    public int getJoystickCount () {
        return joystickDescriptors.length;
    }

    public int getAxisCount (final int joystick) {
        return joystickDescriptors[joystick].getAxisCount();
    }

    public int getButtonCount (final int joystick) {
        return joystickDescriptors[joystick].getButtonCount();
    }

    public double getTimestamp (final int frame) {
        return timestampBuffer[frame];
    }

    public void setTimestamp (final int frame, final double value) {
        timestampBuffer[frame] = value;
    }

    private int getAxisIndex (final int frame, final int joystick, int axis) {
        return frame * totalAxisCount + axisOffsets[joystick] + axis;
    }

    public double getRawAxis (final int frame, final int joystick, final int axis) {
        return axisBuffer[getAxisIndex(frame, joystick, axis)];
    }

    public void setRawAxis (final int frame, final int joystick, final int axis, final double value) {
        axisBuffer[getAxisIndex(frame, joystick, axis)] = value;
    }

    private int getButtonsIndex (final int frame, final int joystick) {
        return frame * joystickDescriptors.length + joystick;
    }

    public boolean getRawButton (final int frame, final int joystick, final int button) {
        return ((buttonBuffer[getButtonsIndex(frame, joystick)] >> (button - 1)) & 1) == 1;
    }

    public void setRawButton (final int frame, final int joystick, final int button, final boolean value) {
        buttonBuffer[getButtonsIndex(frame, joystick)] |= (value ? 1 : 0) << (button - 1);
    }

    public static InputRecording load (final String path) throws IOException {
        try (final DataInputStream in = new DataInputStream(new FileInputStream(path))) {
            final int frameCount = in.readInt();
            System.out.println("frameCount = " + frameCount);
            final int joystickCount = in.readInt();
            System.out.println("joystickCount = " + joystickCount);
            final JoystickDescriptor[] joystickDescriptors = new JoystickDescriptor[joystickCount];
            for (int i = 0; i < joystickCount; ++i) {
                final int axisCount = in.readInt();
                final int buttonCount = in.readInt();
                joystickDescriptors[i] = new JoystickDescriptor(axisCount, buttonCount);
            }

            final InputRecording recording = new InputRecording(frameCount, joystickDescriptors);
            for (int i = 0; i < recording.timestampBuffer.length; ++i) {
                recording.timestampBuffer[i] = in.readDouble();
            }
            for (int i = 0; i < recording.axisBuffer.length; ++i) {
                recording.axisBuffer[i] = in.readDouble();
            }
            for (int i = 0; i < recording.buttonBuffer.length; ++i) {
                recording.buttonBuffer[i] = in.readInt();
            }

            return recording;
        }
    }

    public void save (final String path) throws IOException {
        try (final DataOutputStream out = new DataOutputStream(new FileOutputStream(path))) {
            out.writeInt(frameCount);
            out.writeInt(joystickDescriptors.length);
            for (JoystickDescriptor descriptor : joystickDescriptors) {
                out.writeInt(descriptor.getAxisCount());
                out.writeInt(descriptor.getButtonCount());
            }

            for (int i = 0; i < frameCount + 1; ++i) {
                out.writeDouble(timestampBuffer[i]);
            }
            for (int i = 0; i < frameCount * totalAxisCount; ++i) {
                out.writeDouble(axisBuffer[i]);
            }
            for (int i = 0; i < frameCount * joystickDescriptors.length; ++i) {
                out.writeInt(buttonBuffer[i]);
            }
        }
    }
}