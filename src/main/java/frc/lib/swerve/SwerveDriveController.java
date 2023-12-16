package frc.lib.swerve;

/*
 * Started from https://github.com/mjansen4857/pathplanner (modified version of WPILib controller)
 *
 * Copied here to stablize.
 */

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.lib.trajectory.TrajectoryState;

public class SwerveDriveController {
  private final PIDController xController;
  private final PIDController yController;
  private final ProfiledPIDController rotationController;

  private Translation2d translationError = new Translation2d();
  private Rotation2d rotationError = new Rotation2d();
  private boolean isEnabled = true;

  /**
   * Constructs a HolonomicDriveController
   *
   * @param translationConstants PID constants for the translation PID controllers
   * @param rotationConstants PID constants for the rotation controller
   * @param period Period of the control loop in seconds
   * @param maxModuleSpeed The max speed of a drive module in meters/sec
   * @param driveBaseRadius The radius of the drive base in meters. For swerve drive, this is the
   *     distance from the center of the robot to the furthest module. For mecanum, this is the
   *     drive base width / 2
   */
  public SwerveDriveController(
      PIDController xController,
      PIDController yController,
      ProfiledPIDController rotationController) {
    this.xController = xController;
    this.yController = yController;

    this.rotationController = rotationController;

    this.rotationController.enableContinuousInput(-Math.PI, Math.PI);
  }

  /**
   * Enables and disables the controller for troubleshooting. When calculate() is called on a
   * disabled controller, only feedforward values are returned.
   *
   * @param enabled If the controller is enabled or not
   */
  public void setEnabled(boolean enabled) {
    this.isEnabled = enabled;
  }

  /**
   * Resets the controller based on the current state of the robot
   *
   * @param currentPose Current robot pose
   * @param currentSpeeds Current robot relative chassis speeds
   */
  public void reset(Pose2d currentPose, ChassisSpeeds currentSpeeds) {
    rotationController.reset(
        currentPose.getRotation().getRadians(), currentSpeeds.omegaRadiansPerSecond);
  }

  /**
   * Calculates the next output of the holonomic drive controller
   *
   * @param currentPose The current robot pose
   * @param targetState The desired trajectory state
   * @return The next robot relative output of the path following controller
   */
  public ChassisSpeeds calculate(Pose2d currentPose, TrajectoryState targetState) {
    double xFF = targetState.velocityX;
    double yFF = targetState.velocityY;
    double rotationFF = targetState.angularVelocity;

    this.translationError = targetState.getPose().relativeTo(currentPose).getTranslation();
    this.rotationError = new Rotation2d(targetState.heading).minus(currentPose.getRotation());

    if (!this.isEnabled) {
      return ChassisSpeeds.fromFieldRelativeSpeeds(xFF, yFF, 0, currentPose.getRotation());
    }

    double xFeedback = this.xController.calculate(currentPose.getX(), targetState.x);
    double yFeedback = this.yController.calculate(currentPose.getY(), targetState.y);
    double rotationFeedback =
        this.rotationController.calculate(
            currentPose.getRotation().getRadians(), targetState.heading);

    // TODO: Should the trapezoid look at velocity = 0 or targetState.angularVelocity
    double targetRotationVel =
        rotationController.calculate(
            currentPose.getRotation().getRadians(),
            new TrapezoidProfile.State(targetState.heading, 0)); // targetState.angularVelocity));

    return ChassisSpeeds.fromFieldRelativeSpeeds(
        xFF + xFeedback, yFF + yFeedback, targetRotationVel, currentPose.getRotation());
  }

  /**
   * Get the current positional error between the robot's actual and target positions
   *
   * @return Positional error, in meters
   */
  public double getPositionalError() {
    return translationError.getNorm();
  }
}
