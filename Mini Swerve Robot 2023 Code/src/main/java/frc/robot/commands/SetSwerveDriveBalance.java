package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrive;
import java.util.function.DoubleSupplier;

public class SetSwerveDriveBalance extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final SwerveDrive m_swerveDrive;

  private final DoubleSupplier m_throttleInput, m_strafeInput, m_rotationInput;
  SwerveModuleState[] states;


  // If your going the wrong way then add a negative or remove one from the kp value
  PIDController outputCalculator = new PIDController(0.05, 0, 0);

  /**
   * Creates a new ExampleCommand.
   *
   * @param swerveDriveSubsystem The subsystem used by this command.
   */
  public SetSwerveDriveBalance(
      SwerveDrive swerveDriveSubsystem,
      DoubleSupplier throttleInput,
      DoubleSupplier strafeInput,
      DoubleSupplier rotationInput) {
    m_swerveDrive = swerveDriveSubsystem;
    m_throttleInput = throttleInput;
    m_strafeInput = strafeInput;
    m_rotationInput = rotationInput;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerveDriveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    states =
        new SwerveModuleState[] {
          new SwerveModuleState(0, Rotation2d.fromDegrees(0)),
          new SwerveModuleState(0, Rotation2d.fromDegrees(0)),
          new SwerveModuleState(0, Rotation2d.fromDegrees(0)),
          new SwerveModuleState(0, Rotation2d.fromDegrees(0)),
        };
    m_swerveDrive.setSwerveModuleStates(states, false);
    outputCalculator.setSetpoint(0);
    outputCalculator.setTolerance(2);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double output = outputCalculator.calculate(m_swerveDrive.getPitchDegrees());
    states =
        new SwerveModuleState[] {
          new SwerveModuleState(output, Rotation2d.fromDegrees(0)),
          new SwerveModuleState(output, Rotation2d.fromDegrees(0)),
          new SwerveModuleState(output, Rotation2d.fromDegrees(0)),
          new SwerveModuleState(output, Rotation2d.fromDegrees(0)),
        };
    m_swerveDrive.setSwerveModuleStates(states, false);
    SmartDashboard.putNumber("Balence output", output);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    states =
        new SwerveModuleState[] {
          new SwerveModuleState(0, Rotation2d.fromDegrees(0)),
          new SwerveModuleState(0, Rotation2d.fromDegrees(0)),
          new SwerveModuleState(0, Rotation2d.fromDegrees(0)),
          new SwerveModuleState(0, Rotation2d.fromDegrees(0)),
        };
    m_swerveDrive.setSwerveModuleStates(states, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}