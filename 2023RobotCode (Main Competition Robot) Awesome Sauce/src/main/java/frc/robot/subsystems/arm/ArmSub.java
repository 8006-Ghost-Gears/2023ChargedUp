// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.commands.WaitThenRunCommand;

public class ArmSub extends SubsystemBase {
  /** Creates a new climberSub. */
  CANSparkMax arm;
  private final PIDController pidController = new PIDController(0.1, 0, 0);

  public ArmSub() {
    arm = new CANSparkMax(Constants.arm, MotorType.kBrushless);

    arm.restoreFactoryDefaults();
    arm.setInverted(false);
    arm.setOpenLoopRampRate(0.7); // changes how fast you get 100%
    arm.getEncoder().setPosition(0);
  }

  public void setPower(double armpower) {
    arm.set(armpower);
  }

  public void armDown() {
    arm.set(0.3);
  }

  public void armUp() {
    arm.set(0.3);
  }

  public void stopArm() {
    arm.set(0);
  }

  public void checkPosition() {

    // Set the maximum joystick input value
    double maxJoystickInput = 2;

    // Set the maximum NEO motor output value
    double maxMotorOutput = 1.0;

    //if (arm.getEncoder().getPosition() > -1) {

      double armcommands = -RobotContainer.operator.getRightY(); // A minus could be the difference between not working and working

      double scaledJoystickInput = armcommands * (maxMotorOutput / maxJoystickInput);

      RobotContainer.mArmSub.setPower(Math.signum(scaledJoystickInput) * Math.pow(scaledJoystickInput, 2));
    } 
   // else {

      //double armcommands = RobotContainer.mArmSub.getLeftJoystickUp();

      //double scaledJoystickInput = armcommands * (maxMotorOutput / maxJoystickInput);

      //RobotContainer.mArmSub.setPower(Math.signum(scaledJoystickInput) * Math.pow(scaledJoystickInput, 2));

      public void ArmStowed() {
    
        

        final double kArmTolerance = 2.0;
    
        double targetPosition = 5;
    
        // introduce a delay of 2 seconds
    
            pidController.setSetpoint(targetPosition);
    
            arm.getPIDController().setP(0.1);
            arm.getPIDController().setI(0);
            arm.getPIDController().setD(0);
    
            pidController.setTolerance(kArmTolerance);
    
            arm.getPIDController().setOutputRange(-0.60, 0.60);
    
            arm.getPIDController().setReference(targetPosition, CANSparkMax.ControlType.kPosition);
    
        }      

        public void ArmCubeIntake() {
    
          final double kArmTolerance = 2.0;
          double targetPosition = 76; //TO-DO Find Position
          
          // introduce a delay of 1 second before executing the following code
          new WaitThenRunCommand(2, () -> {
              
              pidController.setSetpoint(targetPosition);
          
              arm.getPIDController().setP(0.1);
              arm.getPIDController().setI(0);
              arm.getPIDController().setD(0);
          
              pidController.setTolerance(kArmTolerance);
          
              arm.getPIDController().setOutputRange(-0.30, 0.30);
          
              arm.getPIDController().setReference(targetPosition, CANSparkMax.ControlType.kPosition);
              
          }).schedule();
      }           

        public void ArmLowScoring() {

          
    
          final double kArmTolerance = 2.0;
      
          double targetPosition = 0; //TO-DO Find Position
      
          // introduce a delay of 2 seconds
      
              pidController.setSetpoint(targetPosition);
      
              arm.getPIDController().setP(0.1);
              arm.getPIDController().setI(0);
              arm.getPIDController().setD(0);
      
              pidController.setTolerance(kArmTolerance);
      
              arm.getPIDController().setOutputRange(-0.60, 0.60);
      
              arm.getPIDController().setReference(targetPosition, CANSparkMax.ControlType.kPosition);
      
          }
    
  

  // To get just the up value on the joystick
  public double getLeftJoystickUp() {
    double leftJoystickY = -1.0 * RobotContainer.operator.getRawAxis(1);
    return Math.max(0.0, leftJoystickY);
  }

  public double getLeftJoystickDown() {
    double leftJoystickY = RobotContainer.operator.getRawAxis(1);
    return Math.max(0.0, leftJoystickY);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Arm", arm.getEncoder().getPosition());
  }
}