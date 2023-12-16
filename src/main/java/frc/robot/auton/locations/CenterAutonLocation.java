package frc.robot.auton.locations;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.auton.AutonLocation;

public class CenterAutonLocation extends AutonLocation {

  public static Pose2d get() {
    return new Pose2d(1.82, 3.26, new Rotation2d());
  }
}
