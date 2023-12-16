package frc.lib.trajectory;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.Interpolatable;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class TrajectoryState implements Interpolatable<TrajectoryState> {
  public final double timestamp;
  public final double x;
  public final double y;
  public final double heading;

  public final double velocityX;
  public final double velocityY;
  public final double angularVelocity;

  public TrajectoryState(
      double timestamp,
      double x,
      double y,
      double heading,
      double velocityX,
      double velocityY,
      double angularVelocity) {
    this.timestamp = timestamp;
    this.x = x;
    this.y = y;
    this.heading = heading;
    this.velocityX = velocityX;
    this.velocityY = velocityY;
    this.angularVelocity = angularVelocity;
  }

  public double getTimestamp() {
    return timestamp;
  }

  public Pose2d getPose() {
    return new Pose2d(x, y, Rotation2d.fromRadians(heading));
  }

  public ChassisSpeeds getChassisSpeeds() {
    return new ChassisSpeeds(velocityX, velocityY, angularVelocity);
  }

  @Override
  public TrajectoryState interpolate(TrajectoryState endValue, double t) {
    double scale = (getTimestamp() - t) / (endValue.getTimestamp() - t);
    var interp_pose = getPose().interpolate(endValue.getPose(), scale);

    return new TrajectoryState(
        timestamp,
        interp_pose.getX(),
        interp_pose.getY(),
        interp_pose.getRotation().getRadians(),
        MathUtil.interpolate(this.velocityX, endValue.velocityX, scale),
        MathUtil.interpolate(this.velocityY, endValue.velocityY, scale),
        MathUtil.interpolate(this.angularVelocity, endValue.angularVelocity, scale));
  }

  public double[] asArray() {
    return new double[] {
      timestamp, x, y, heading, velocityX, velocityY, angularVelocity,
    };
  }
}
