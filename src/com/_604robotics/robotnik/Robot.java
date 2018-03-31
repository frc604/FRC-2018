package com._604robotics.robotnik;

import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.tables.ITable;

import java.io.ByteArrayInputStream;
import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.util.ArrayList;
import java.util.Base64;
import java.util.List;
import java.util.function.Supplier;
import java.util.stream.Collectors;
import java.util.zip.InflaterInputStream;

import com._604robotics.robotnik.utils.Pair;

@SuppressWarnings("deprecation")
public abstract class Robot extends SampleRobot {
    public static double DEFAULT_REPORT_INTERVAL = 5;

    private static final Logger logger = new Logger(Robot.class);

    private final ITable table = NetworkTable.getTable("robotnik");

    private final List<Module> modules = new ArrayList<>();
    private final IterationTimer iterationTimer;

    private final List<Pair<String, Coordinator>> systems = new ArrayList<>();

    private Coordinator autonomousMode;
    private Coordinator teleopMode;
    private Coordinator testMode;

    private boolean sealed = false;

    public Robot () {
        this(DEFAULT_REPORT_INTERVAL);
    }

    public Robot (double reportInterval) {
        iterationTimer = new IterationTimer(reportInterval);
        updateModuleList();
    }

    private void assertUnsealed () {
        if (sealed) {
            throw new IllegalStateException("Robot is sealed");
        }
    }

    protected <T extends Module> T addModule (T module) {
        assertUnsealed();
        modules.add(module);
        updateModuleList();
        return module;
    }

    private void updateModuleList () {
        table.putString("moduleList", modules.stream()
                .map(Module::getName)
                .collect(Collectors.joining(",")));
    }

    protected void addSystem (String name, Coordinator system) {
        assertUnsealed();
        systems.add(new Pair<>(name, system));
    }

    protected void addSystem (Class<?> klass, Coordinator system) {
        assertUnsealed();
        addSystem(klass.getSimpleName(), system);
    }

    protected void setAutonomousMode (Coordinator autonomousMode) {
        assertUnsealed();
        this.autonomousMode = autonomousMode;
    }

    protected void setTeleopMode (Coordinator teleopMode) {
        assertUnsealed();
        this.teleopMode = teleopMode;
    }

    protected void setTestMode (Coordinator testMode) {
        assertUnsealed();
        this.testMode = testMode;
    }

    @Override
    protected void robotInit () {
        sealed = true;
        for (Module module : modules) {
            module.seal();
        }

        printBanner();
    }

    @Override
    public void autonomous () {
        loop("Autonomous", autonomousMode, () -> isEnabled() && isAutonomous());
    }

    @Override
    public void operatorControl () {
        loop("Teleop", teleopMode, () -> isEnabled() && isOperatorControl());
    }

    @Override
    public void test () {
        loop("Test", testMode, () -> isEnabled() && isTest());
    }

    @Override
    protected void disabled () {
        loop("Disabled", null, this::isDisabled);
    }

    private void loop (String name, Coordinator mode, Supplier<Boolean> active) {
        logger.info(name + " mode begin");

        for (Module module : modules) {
            Reliability.swallowThrowables(module::begin, "Error in begin() of module " + module.getName());
        }

        if (mode != null) {
            Reliability.swallowThrowables(mode::start, "Error starting " + name + " mode");
            iterationTimer.start();
        }

        for (Pair<String, Coordinator> system : systems) {
            Reliability.swallowThrowables(system.getValue()::start, "Error starting system " + system.getKey());
        }

        while (active.get()) {
            for (Module module : modules) {
                module.prepare();
            }

            if (mode != null) {
                Reliability.swallowThrowables(mode::execute, "Error executing " + name + " mode");
            }

            for (Pair<String, Coordinator> system : systems) {
                Reliability.swallowThrowables(system.getValue()::execute, "Error executing system " + system.getKey());
            }

            for (Module module : modules) {
                module.update();

                if (mode != null) {
                    module.execute();
                }

                Reliability.swallowThrowables(module::run, "Error in run() of module " + module.getName());
            }

            if (mode != null) {
                iterationTimer.sample(
                        loopTime -> logger.info("Loop time: " + loopTime * 1000 + " ms"));
            }
        }

        if (mode != null) {
            iterationTimer.stop();
            Reliability.swallowThrowables(mode::stop, "Error stopping " + name + " mode");
        }

        for (Pair<String, Coordinator> system : systems) {
            Reliability.swallowThrowables(system.getValue()::stop, "Error stopping system " + system.getKey());
        }

        for (Module module : modules) {
            if (mode != null) {
                module.terminate();
            }

            Reliability.swallowThrowables(module::end, "Error in end() of module " + module.getName());
        }

        logger.info(name + " mode end");
    }

    private void printBanner () {
        byte[] compRaw=Base64.getDecoder().decode("eNqlVsGO3CAMvfMV3DIrZPtu9VNGwj1sL1UVaXuoVsrH1zYkgYTMrGbJikDAj4f97NkQQrxuwiwM/TcGYJBwbVEMkLCxIbE14EtLao/RIwARtW9AzHpsm+LzJhRWmP42+NyWpUOCFWnWu+5LSM+RGqZIzPN6H2B3XOFCXwBSc6iW5ri0e7V4zD5Dki0o6krYuR7munE9HBoXz/HoMeJqXhhKgupb3oArpiTHEY0h09HFtBEBN9x4xqPWXAmAQV0NKEPdNBHuPYw81IESUm5yZrUSr8dgGeA+HwcCiIvzbTBUT3E2d3M+3bRGlXBV198vKPOhaqu6dhm83DA5K6GetAYXrbM8jfzZHIOfGglA15iu9gS0GChamqEEVx/ybBINZnGhvnlnDYlqnNm2wOzCcUNvs2lW5JjTCVo4zSqpaRdbOKZjBZFThSqJwKmFs3jXBGjgZuxqTex1vlGrCPqW+t4Q6omnQre6XrzmDAFU0+sxCcNAh02NHgHskpR5YH8Qkrjvk4nCWi+XYZHfy8TDQp0uqjxe5Fe348HPUkuxZ6yOg/DwR5CrhC2OFQjta3hyl+IavzbHUsq+k4YwTd9BwEkhVIBvL0OQYWhPjxDycJx9fMsULwB0g/3luMQl2951XFZu+U1nvjC0X+KUFTzHu43iPdZxzmW+GBZFujS3pxyyeL+Ol7g+Mf4wL8bpbK274l0J05LRaPjYerfP/tyzkfJEzxrKcxR+ffz88/5v/vj9NIyTQUymh/BqIPUfBZgUJLxq7xRes765A0II/wGVOKOq");
        InputStream compInflate=new InflaterInputStream(new ByteArrayInputStream(compRaw));
        ByteArrayOutputStream finalStream=new ByteArrayOutputStream();
        int copyInt=0;
        while(copyInt!=-1) {
        	try {
				copyInt=compInflate.read();
			} catch (IOException e) {
				e.printStackTrace();
				copyInt=-1;
			}
        	if (copyInt==-1) {
        		break;
        	}
        	finalStream.write(copyInt);
        }
        System.out.println(new String(finalStream.toByteArray()));
    }
}