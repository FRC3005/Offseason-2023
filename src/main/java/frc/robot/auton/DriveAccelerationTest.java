package frc.robot.auton;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.auton.AutonCommandBase;
import frc.lib.vendor.sensor.ADIS16470;
import frc.robot.subsystems.drive.DriveSubsystem;

public class DriveAccelerationTest extends AutonCommandBase {
  private final DriveSubsystem m_drive;
  private final ADIS16470 m_gyro;
  private static final double kSprintDistancesMeters = 1.0;
  private double m_angleGain = 0.0;
  private static final double kDecelerationRate = 1.0;
  private final SlewRateLimiter m_limiter = new SlewRateLimiter(kDecelerationRate);

  public DriveAccelerationTest(DriveSubsystem drive, ADIS16470 gyro) {
    m_drive = drive;
    m_gyro = gyro;

    addCommands(
        // Reset the heading to 0.0
        Commands.runOnce(() -> m_drive.setHeading(0.0)),

        // Drive full speed no ramping until we either hit the switch, or reach max distance
        // Keep the bot straight (0 degrees) using the gyro
        Commands.run(
                () -> {
                  m_drive.drive(1.0, 0.0, m_gyro.getAngle() * m_angleGain, false);
                })
            .until(() -> m_drive.getPose().getX() > kSprintDistancesMeters),

        // Next, ramp throttle to zero at some rate
        Commands.runOnce(() -> m_limiter.reset(1.0)),
        Commands.run(() -> m_drive.drive(m_limiter.calculate(0.0), 0.0, 0.0, false)));
  }
}
