// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intakecommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class WristCommand extends CommandBase {
  /** Creates a new driveCommand. */
  public WristCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.mWristSub);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.

  public void execute() {
    RobotContainer.mWristSub.checkPosition();
  }


}

