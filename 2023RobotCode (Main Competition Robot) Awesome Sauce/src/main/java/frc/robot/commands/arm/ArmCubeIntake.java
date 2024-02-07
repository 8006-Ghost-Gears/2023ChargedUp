// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class ArmCubeIntake extends CommandBase {
  /** Creates a new ShoulderDownCommand. */
  public ArmCubeIntake() {
    addRequirements(RobotContainer.mShoulderSub);
    addRequirements(RobotContainer.mWristSub);
    addRequirements(RobotContainer.mArmSub);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.mShoulderSub.ShoulderCubeIntake();
    RobotContainer.mArmSub.ArmCubeIntake();
    RobotContainer.mWristSub.WristCubeIntake();
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
