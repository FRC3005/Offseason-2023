// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

// import java.util.function.Supplier;

// import com.pathplanner.lib.PathPlannerTrajectory;
// import com.pathplanner.lib.commands.PPSwerveControllerCommand;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.constants.Drivetrain;

public class PathPlannerCommands {
  /**
   * Create a trajectory following command. Note that the beginning and end states of the command
   * are not necessarily 0 speed.
   *
   * @param trajectory PathPlanner trajectory
   * @param xController PID Controller for the X direction (left/right)
   * @param yController PID Contorller for the Y direction (forward/back)
   * @param thetaController Turning PID Controller for rotation (CCW Positive)
   * @return Command to be scheduled
   */
  // public Command trajectoryFollowerCommand(
  //     PathPlannerTrajectory trajectory,
  //     PIDController xController,
  //     PIDController yController,
  //     PIDController thetaController) {
  //   return trajectoryFollowerCommand(
  //       trajectory, this::getPose, xController, yController, thetaController);
  // }

  // /**
  //  * Create a trajectory following command. Note that the beginning and end states of the command
  //  * are not necessarily 0 speed.
  //  *
  //  * @param trajectory PathPlanner trajectory
  //  * @param xController PID Controller for the X direction (left/right)
  //  * @param yController PID Contorller for the Y direction (forward/back)
  //  * @param thetaController Turning PID Controller for rotation (CCW Positive)
  //  * @return Command to be scheduled
  //  */
  // public Command trajectoryFollowerCommand(
  //     PathPlannerTrajectory trajectory,
  //     Supplier<Pose2d> poseSupplier,
  //     PIDController xController,
  //     PIDController yController,
  //     PIDController thetaController) {
  //   Command swCommand =
  //       new PPSwerveControllerCommand(
  //           trajectory,
  //           poseSupplier,
  //           m_kinematics,
  //           xController,
  //           yController,
  //           thetaController,
  //           (states) -> setModuleStates(states),
  //           true,
  //           this);
  //   return new InstantCommand(() -> m_field.getObject("Trajectory").setTrajectory(trajectory))
  //       .alongWith(swCommand);
  // }

  // /**
  //  * Add functions to log telemetry for any pathing to NT. Keep these separate from the 'bind'
  //  * function so they only run when a command is actually active.
  //  */
  // private void configurePathPlannerTelemetry() {
  //   PPSwerveControllerCommand.setLoggingCallbacks(
  //       this::PPLogTrajectory, this::PPLogTargetPose, this::PPLogSetpoint, this::PPLogError);
  // }

  // private FieldObject2d m_trajectory = m_field.getObject("Active Trajectory");

  // private void PPLogTrajectory(PathPlannerTrajectory trajectory) {
  //   m_trajectory.setTrajectory(trajectory);
  // }

  // private FieldObject2d m_targetPose = m_field.getObject("Target Pose");

  // private void PPLogTargetPose(Pose2d pose) {
  //   m_targetPose.setPose(pose);
  // }

  // private void PPLogSetpoint(ChassisSpeeds setpoint) {
  //   SmartDashboard.putNumber("PPSwerveControllerCommand/vxSetpoint", setpoint.vxMetersPerSecond);
  //   SmartDashboard.putNumber("PPSwerveControllerCommand/vySetpoint", setpoint.vyMetersPerSecond);
  //   SmartDashboard.putNumber(
  //       "PPSwerveControllerCommand/rotSetpoint", setpoint.omegaRadiansPerSecond);
  // }

  // private void PPLogError(Translation2d translationError, Rotation2d rotationError) {
  //   SmartDashboard.putNumber("PPSwerveControllerCommand/xErrorMeters", translationError.getX());
  //   SmartDashboard.putNumber("PPSwerveControllerCommand/yErrorMeters", translationError.getY());
  //   SmartDashboard.putNumber(
  //       "PPSwerveControllerCommand/rotationErrorRadians", rotationError.getRadians());
  // }

  // static public Command trajectoryFollowerCommand(PathPlannerTrajectory trajectory) {
  //   Drivetrain.kThetaController.enableContinuousInput(-Math.PI, Math.PI);
  //   return trajectoryFollowerCommand(
  //       trajectory, Drivetrain.kXController, Drivetrain.kYController,
  // Drivetrain.kThetaController);
  // }

  // public static Command trajectoryFollowerCommand(
  //     PathPlannerTrajectory trajectory, Supplier<Pose2d> poseSupplier) {
  //   Drivetrain.kThetaController.enableContinuousInput(-Math.PI, Math.PI);
  //   return trajectoryFollowerCommand(
  //       trajectory,
  //       poseSupplier,
  //       Drivetrain.kXController,
  //       Drivetrain.kYController,
  //       Drivetrain.kThetaController);
  // }
}
