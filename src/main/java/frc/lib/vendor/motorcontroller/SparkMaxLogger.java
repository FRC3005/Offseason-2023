package frc.lib.vendor.motorcontroller;

import java.util.function.Consumer;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import frc.lib.telemetrystream.StreamedDouble;
import frc.lib.vendor.motorcontroller.stream.SparkMaxStream;

public class SparkMaxLogger {
    private final SparkMaxStream m_stream;
    private static final String NT_PREFIX = "/sparkmaxlog/";

    private Consumer<StreamedDouble> LoggedStreamFactory(String ntEntryName, DataLog log) {
        return (streamedDouble) -> {
            String fullNtPath = NT_PREFIX + String.valueOf(streamedDouble.deviceId) + "/" + ntEntryName;
            new DoubleLogEntry(log, fullNtPath).append(streamedDouble.value, streamedDouble.timestamp);
        };
    }

    public SparkMaxLogger(DataLog log) {
        m_stream = new SparkMaxStream();

        m_stream.appliedOutputConsumer(LoggedStreamFactory("appliedOutput", log));
        m_stream.stickyFaultsConsumer((streamedInteger) -> {
            String fullNtPath = NT_PREFIX + String.valueOf(streamedInteger.deviceId) + "/" + "stickyFaults";
            new DoubleLogEntry(log, fullNtPath).append(streamedInteger.value, streamedInteger.timestamp);
        });
        m_stream.currentConsumer(LoggedStreamFactory("outputCurrent", log));
        m_stream.velocityConsumer(LoggedStreamFactory("velocity", log));
        m_stream.positionConsumer(LoggedStreamFactory("position", log));
        m_stream.supplyVoltageConsumer(LoggedStreamFactory("supplyVoltage", log));
    }

    public void start() {
        m_stream.start();
    }

    public void stop() {
        m_stream.stop();
    }
}
