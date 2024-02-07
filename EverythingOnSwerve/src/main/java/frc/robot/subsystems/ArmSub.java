// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class ArmSub extends SubsystemBase {
  /** Creates a new climberSub. */
  CANSparkMax arm;
  
  public ArmSub() {
    arm = new CANSparkMax(Constants.arm,MotorType.kBrushless);
    
    arm.restoreFactoryDefaults();
    arm.setInverted(true);
    arm.setOpenLoopRampRate(0.9); //changes how fast you get 100%
    arm.getEncoder().setPosition(0);
  }

  public void setPower(double armpower){
    arm.set(armpower);
  }
  
  public void checkPosition(double armcommands){
    if(arm.getEncoder().getPosition() > -1) {
      armcommands = -RobotContainer.operator.getLeftY();

      RobotContainer.mArmSub.setPower(Math.signum(armcommands)*Math.pow(armcommands, 2));
    }
    else {
      armcommands = RobotContainer.mArmSub.getLeftJoystickUp();
      RobotContainer.mArmSub.setPower(Math.signum(armcommands)*Math.pow(armcommands, 2));
      
    }
  }

  //To get just the up value on the joystick
  public double getLeftJoystickUp() {
    double leftJoystickY = -1.0 * RobotContainer.operator.getRawAxis(1);
    return Math.max(0.0, leftJoystickY);
  }

  public double getLeftJoystickDown() {
    double leftJoystickY = RobotContainer.operator.getRawAxis(1);
    return Math.max(0.0, leftJoystickY);
  }
 /*  public void stopArm(double armcommands){
    arm.set(0);
  }
*/

/*  public void setClimber(){
    if(climberL.getEncoder().getPosition() < 104) {
    climberL.set(1);
    } else {
      climberL.set(0);
    }
    if(climberR.getEncoder().getPosition() < 104) {
      climberR.set(1);
    } else {
      climberR.set(0);
    }
}
*/

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Arm", arm.getEncoder().getPosition());
  }
}
