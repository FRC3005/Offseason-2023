package frc.lib.vendor.motorcontroller;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.IntegerLogEntry;
import frc.lib.telemetrystream.StreamedDouble;
import frc.lib.telemetrystream.StreamedInteger;
import frc.lib.vendor.motorcontroller.stream.SparkMaxStream;
import java.util.function.Consumer;

public class SparkMaxLogger {
  private final SparkMaxStream m_stream;
  private static final String NT_PREFIX = "/sparkmaxlog/";
  private final int m_deviceId;
  private final DataLog m_datalog;

  private Consumer<StreamedDouble> LoggedStreamDoubleFactory(String ntEntryName) {
    String fullNtPath = NT_PREFIX + String.valueOf(m_deviceId) + "/" + ntEntryName;
    DoubleLogEntry logEntry = new DoubleLogEntry(m_datalog, fullNtPath);
    return (streamedDouble) -> {
      logEntry.append(streamedDouble.value, streamedDouble.timestamp);
    };
  }

  private Consumer<StreamedInteger> LoggedIntegerStreamFactory(String ntEntryName) {
    String fullNtPath = NT_PREFIX + String.valueOf(m_deviceId) + "/" + ntEntryName;
    IntegerLogEntry logEntry = new IntegerLogEntry(m_datalog, fullNtPath);
    return (streamedDouble) -> {
      logEntry.append(streamedDouble.value, streamedDouble.timestamp);
    };
  }

  public SparkMaxLogger(CANSparkMax sparkMax, DataLog log) {
    m_deviceId = sparkMax.getDeviceId();
    m_datalog = log;
    m_stream =
        new SparkMaxStream(m_deviceId)
            .appliedOutputConsumer(LoggedStreamDoubleFactory("appliedOutput"))
            .stickyFaultsConsumer(LoggedIntegerStreamFactory("stickyFaults"))
            .currentConsumer(LoggedStreamDoubleFactory("outputCurrent"))
            .velocityConsumer(LoggedStreamDoubleFactory("velocity"))
            .positionConsumer(LoggedStreamDoubleFactory("position"))
            .supplyVoltageConsumer(LoggedStreamDoubleFactory("supplyVoltage"));
  }

  public void start() {
    m_stream.start();
  }

  public void stop() {
    m_stream.stop();
  }
}
