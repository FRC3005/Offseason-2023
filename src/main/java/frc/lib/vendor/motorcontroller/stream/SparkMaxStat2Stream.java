package frc.lib.vendor.motorcontroller.stream;

import frc.lib.telemetrystream.StreamedDouble;
import frc.lib.telemetrystream.StreamedFrame;
import java.util.function.Consumer;

public class SparkMaxStat2Stream implements StreamedFrame {
  private final SparkMaxFrames.Status2In frame = new SparkMaxFrames.Status2In();
  private Consumer<StreamedDouble> m_positionConsumer = null;

  public SparkMaxStat2Stream positionConsumer(Consumer<StreamedDouble> consumer) {
    m_positionConsumer = consumer;
    return this;
  }

  @Override
  public void processOnMatch(int arbid, long timestamp, byte[] data, int length) {
    if (((arbid & 0x1FFFFFC0) != 0x2051880) || length < 8) {
      return;
    }

    frame.Deserialize(data);

    if (m_positionConsumer != null) {
      m_positionConsumer.accept(new StreamedDouble(timestamp, arbid & 0x3F, frame.getPosition()));
    }
  }
}
