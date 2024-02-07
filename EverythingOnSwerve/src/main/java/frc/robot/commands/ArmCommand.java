// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class ArmCommand extends CommandBase {
  /** Creates a new driveCommand. */
  public ArmCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.mArmSub);
    double armcommands = 0;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.

  public void execute() {
 
    double armcommands = 0;

    // leftSide -= RobotContainer.driver.getRawAxis(4);
    // rightSide -= RobotContainer.driver.getRawAxis(4);

    // leftSide += RobotContainer.driver.getRawAxis(3);
    // rightSide += RobotContainer.driver.getRawAxis(3);

    // double turn = RobotContainer.driver.getRawAxis(0);

    // leftSide -= turn;
    // rightSide += turn;
    
    
    armcommands = RobotContainer.operator.getLeftY(); //A minus could be the difference between not working and working

    RobotContainer.mArmSub.setPower(Math.signum(armcommands)*Math.pow(armcommands, 2));

    RobotContainer.mArmSub.checkPosition(armcommands);

  }


}

