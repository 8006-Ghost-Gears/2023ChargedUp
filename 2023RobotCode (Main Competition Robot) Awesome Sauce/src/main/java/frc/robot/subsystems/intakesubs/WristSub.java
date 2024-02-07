// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intakesubs;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.CANEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.WaitThenRunCommand;

public class WristSub extends SubsystemBase {
  /** Creates a new climberSub. */
 CANSparkMax wrist;
private final PIDController pidController = new PIDController(0.1, 0, 0);
  
  public WristSub() {
    wrist = new CANSparkMax(Constants.wrist,MotorType.kBrushless);
    
    wrist.restoreFactoryDefaults();
    wrist.setInverted(false);
    wrist.getEncoder().setPosition(0);

  }

  public void setPower(double wristPower) {
    wrist.set(wristPower);
  }

  public void WristUp(double speed) {
    wrist.set(speed);
  }
  public void WristDown(double speed) {
    wrist.set(-speed);
  }

  public void checkPosition() {

    // Set the maximum joystick input value
    double maxJoystickInput = 3.75;

    // Set the maximum NEO motor output value
    double maxMotorOutput = 1.0;

    //if (arm.getEncoder().getPosition() > -1) {

      double wirstcommands = -RobotContainer.operator.getLeftY(); // A minus could be the difference between not working and working

      double scaledJoystickInput = wirstcommands * (maxMotorOutput / maxJoystickInput);

      RobotContainer.mWristSub.setPower(Math.signum(scaledJoystickInput) * Math.pow(scaledJoystickInput, 2));
    } 

  public void WristStowed() {
    
    final double kWristTolerance = 2.0;

    double targetPosition = 1.5;

    // introduce a delay of 2 seconds

        pidController.setSetpoint(targetPosition);

        wrist.getPIDController().setP(0.1);
        wrist.getPIDController().setI(0);
        wrist.getPIDController().setD(0);

        pidController.setTolerance(kWristTolerance);

        wrist.getPIDController().setOutputRange(-0.30, 0.30);

        wrist.getPIDController().setReference(targetPosition, CANSparkMax.ControlType.kPosition);

    }

    public void WristCubeIntake() {
    
      final double kWristTolerance = 2.0;
  
      double targetPosition = 7.3; //TO-DO Find Position
  
      // introduce a delay of 1 second before executing the following code
      new WaitThenRunCommand(2, () -> {
          pidController.setSetpoint(targetPosition);
  
          wrist.getPIDController().setP(0.1);
          wrist.getPIDController().setI(0);
          wrist.getPIDController().setD(0);
  
          pidController.setTolerance(kWristTolerance);
  
          wrist.getPIDController().setOutputRange(-0.30, 0.30);
  
          wrist.getPIDController().setReference(targetPosition, CANSparkMax.ControlType.kPosition);
  
        }).schedule();
      }  

      public void WristLowScoring() {
    
        final double kWristTolerance = 2.0;
    
        double targetPosition = 0; //TO-DO Find Position
    
        // introduce a delay of 2 seconds
    
            pidController.setSetpoint(targetPosition);
    
            wrist.getPIDController().setP(0.1);
            wrist.getPIDController().setI(0);
            wrist.getPIDController().setD(0);
    
            pidController.setTolerance(kWristTolerance);
    
            wrist.getPIDController().setOutputRange(-0.40, 0.40);
    
            wrist.getPIDController().setReference(targetPosition, CANSparkMax.ControlType.kPosition);
    
        }


/* In this code, we first get the current position of the shoulder encoder and store it in 
the position variable. Then, we check the position and use the WaitThenRunCommand to schedule 
the appropriate action with a 1-second delay. The WaitThenRunCommand is a custom command that 
extends InstantCommand and takes a delay time (in seconds) and a Runnable as parameters. When 
it is scheduled, it waits for the specified delay time and then runs the Runnable. */ 
  

public void stopWrist(){
  wrist.set(0); // elijah allen is not the best button boy, but his sister thoooooooo ;) Jonah:)
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Wrist", wrist.getEncoder().getPosition());

  }
}

