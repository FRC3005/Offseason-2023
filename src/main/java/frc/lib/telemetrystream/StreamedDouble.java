package frc.lib.telemetrystream;

public final class StreamedDouble {
  /**
   * Create a timestamped value.
   *
   * @param timestamp timestamp in local time base
   * @param deviceId the device Id of the frame.
   * @param value value
   */
  public StreamedDouble(long timestamp, int deviceId, double value) {
    this.timestamp = timestamp;
    this.deviceId = deviceId;
    this.value = value;
  }

  /** Timestamp in local time base. */
  public final long timestamp;

  /** Value. */
  public final double value;

  /** Device ID of the frame (FRC Device ID) */
  public final int deviceId;
}
