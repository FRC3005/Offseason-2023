package frc.lib.telemetrystream;

// Not used, using TimestampedDataType classes from NT/wpilog instead.
public class TimestampedData<T> {
  public final long timestamp;
  public final T data;

  public TimestampedData(long timestamp, T data) {
    this.timestamp = timestamp;
    this.data = data;
  }
}
