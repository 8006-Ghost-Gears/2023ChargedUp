// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class TurnTableCommand extends CommandBase {
  /** Creates a new driveCommand. */
  public TurnTableCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.mTurnTableSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double turntablecommands = 0;

    // leftSide -= RobotContainer.driver.getRawAxis(4);
    // rightSide -= RobotContainer.driver.getRawAxis(4);

    // leftSide += RobotContainer.driver.getRawAxis(3);
    // rightSide += RobotContainer.driver.getRawAxis(3);

    // double turn = RobotContainer.driver.getRawAxis(0);

    // leftSide -= turn;
    // rightSide += turn;
    
    
    turntablecommands = RobotContainer.operator.getRightX(); //A minus could be the difference between not working and working


    RobotContainer.mTurnTableSub.setPower(Math.signum(turntablecommands)*Math.pow(turntablecommands, 2));
  }
}
