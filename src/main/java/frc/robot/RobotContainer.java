// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.IntegerLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.util.ThreadUtils;
import frc.lib.vendor.motorcontroller.SparkMaxLogger;
import frc.robot.subsystems.ExampleSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  private final DataLog m_datalog;
  private final SparkMaxLogger m_sparkMaxLogger;
  private final CANSparkMax m_sparkMax;
  private final DoubleLogEntry m_slowVelocityLogger;
  private final DoubleLogEntry m_slowPositionLogger;
  private final DoubleLogEntry m_slowSupplyVoltageLogger;
  private final IntegerLogEntry m_monotonicTimestampLogger;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_datalog = DataLogManager.getLog();
    m_sparkMax = new CANSparkMax(1, MotorType.kBrushless);
    m_sparkMaxLogger = new SparkMaxLogger(m_sparkMax, "myMax", m_datalog);
    m_slowVelocityLogger = new DoubleLogEntry(m_datalog, "/slowmax/1/velocity");
    m_slowPositionLogger = new DoubleLogEntry(m_datalog, "/slowmax/1/position");
    m_slowSupplyVoltageLogger = new DoubleLogEntry(m_datalog, "/slowmax/1/supplyVoltage");
    m_monotonicTimestampLogger = new IntegerLogEntry(m_datalog, "/sparkmax/monotonic_timestamp");

    // Configure the trigger bindings
    configureBindings();

    m_sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 1);
    m_sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 1);
    m_sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 1);
    m_sparkMax.setOpenLoopRampRate(5.0);
    m_sparkMax.setIdleMode(IdleMode.kCoast);

    m_sparkMax.getEncoder().setAverageDepth(2);
    m_sparkMax.getEncoder().setMeasurementPeriod(8);

    ThreadUtils.sleep(300);
    m_sparkMax.burnFlash();
    ThreadUtils.sleep(300);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return Commands.run(
            () -> {
              m_sparkMax.set(0.2);
            })
        .alongWith(
            Commands.repeatingSequence(
                new InstantCommand(
                    () -> {
                      m_sparkMaxLogger.start();
                      long timestamp = System.nanoTime() / 1000000;
                      m_slowVelocityLogger.append(m_sparkMax.getEncoder().getVelocity(), timestamp);
                      m_slowPositionLogger.append(m_sparkMax.getEncoder().getPosition(), timestamp);
                      m_slowSupplyVoltageLogger.append(m_sparkMax.getBusVoltage(), timestamp);
                      m_monotonicTimestampLogger.append(System.nanoTime() / 1000, timestamp);
                      m_sparkMaxLogger.poll();
                    })))
        .raceWith(Commands.waitSeconds(5))
        .andThen(
            () -> {
              m_sparkMax.set(0);
              m_sparkMaxLogger.stop();
            });
  }

  public void disabledInit() {}

  public void disabledPeriodic() {}

  public void autonInit() {}

  public void teleopInit() {}

  public void testModeInit() {}

  public void testModePeriodic() {}
}
