package frc.lib.vendor.motorcontroller.stream;

import frc.lib.telemetrystream.StreamedDouble;
import frc.lib.telemetrystream.StreamedFrame;
import java.util.function.Consumer;

public class SparkMaxStat1Stream implements StreamedFrame {
  private final SparkMaxFrames.Status1In frame = new SparkMaxFrames.Status1In();
  private Consumer<StreamedDouble> m_velocityConsumer = null;
  private Consumer<StreamedDouble> m_currentConsumer = null;
  private Consumer<StreamedDouble> m_voltageConsumer = null;

  public SparkMaxStat1Stream velocityConsumer(Consumer<StreamedDouble> consumer) {
    m_velocityConsumer = consumer;
    return this;
  }

  public SparkMaxStat1Stream currentConsumer(Consumer<StreamedDouble> consumer) {
    m_currentConsumer = consumer;
    return this;
  }

  public SparkMaxStat1Stream supplyVoltageConsumer(Consumer<StreamedDouble> consumer) {
    m_voltageConsumer = consumer;
    return this;
  }

  private void runConsumer(
      Consumer<StreamedDouble> consumer, long timestamp, int deviceId, double val) {
    if (consumer == null) {
      return;
    }

    consumer.accept(new StreamedDouble(timestamp, deviceId, val));
  }

  @Override
  public void processOnMatch(int arbid, long timestamp, byte[] data, int length) {
    if (((arbid & 0x1FFFFFC0) != 0x2051840) || length < 8) {
      return;
    }

    int deviceId = arbid & 0x3F;

    frame.Deserialize(data);
    runConsumer(m_velocityConsumer, timestamp, deviceId, frame.getVelocity());
    runConsumer(m_currentConsumer, timestamp, deviceId, frame.getOutputCurrent());
    runConsumer(m_voltageConsumer, timestamp, deviceId, frame.getBusVoltage());
  }
}
