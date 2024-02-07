// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class HandSub extends SubsystemBase {
  /** Creates a new climberSub. */
  CANSparkMax hand;
  
  public HandSub() {
    hand = new CANSparkMax(Constants.hand,MotorType.kBrushless);
    
    hand.restoreFactoryDefaults();
    hand.setInverted(false);
    hand.getEncoder().setPosition(0);

  }


public void handOpen(){
  hand.set(0.05);
}

public void handClose() {
  if(hand.getEncoder().getPosition() > -1) {
    hand.set(-0.05);
  }
  else {
      hand.set(0);
    }
}

public void stopHand(){
  hand.set(0);
}


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
    SmartDashboard.putNumber("Hand", hand.getEncoder().getPosition());
  }
}
