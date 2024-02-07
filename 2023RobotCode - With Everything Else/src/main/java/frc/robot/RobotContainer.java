// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import java.io.IOException;
import java.nio.file.Path;
import java.util.HashMap;

import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.arm.ArmCommand;
import frc.robot.commands.wristandhand.CloseHandCommand;
import frc.robot.commands.wristandhand.OpenHandCommand;
import frc.robot.commands.swerve.SetSwerveDrive;
import frc.robot.commands.swerve.SetSwerveDriveBalance;
import frc.robot.commands.swerve.SetSwerveCoastMode;
import frc.robot.commands.arm.ShoulderDownCommand;
import frc.robot.commands.arm.ShoulderUpCommand;
import frc.robot.commands.arm.TurnTableCommand;
import frc.robot.commands.wristandhand.WristTurnLeftCommand;
import frc.robot.commands.wristandhand.WristTurnRightCommand;
import frc.robot.simulation.FieldSim;
import frc.robot.subsystems.arm.ArmSub;
import frc.robot.subsystems.wristandhand.HandSub;
import frc.robot.subsystems.arm.ShoulderSub;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.arm.TurnTableSub;
import frc.robot.subsystems.wristandhand.WristSub;
import frc.robot.commands.arm.Stowed;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  private final DataLog m_logger = DataLogManager.getLog();

  // The robot's subsystems and commands are defined here...

  private final SwerveDrive m_swerveDrive = new SwerveDrive();

  private final SendableChooser<Command> m_autoChooser = new SendableChooser<Command>();

  private final FieldSim m_fieldSim = new FieldSim(m_swerveDrive);

  HashMap<String, Command> m_eventMap = new HashMap<>();
  private SwerveAutoBuilder m_autoBuilder;

  static Joystick leftJoystick = new Joystick(Constants.USB.leftJoystick);
  static Joystick rightJoystick = new Joystick(Constants.USB.rightJoystick);
  static XboxController xBoxController = new XboxController(Constants.USB.xBoxController);
  static PS4Controller testController = new PS4Controller(Constants.USB.testController);

  public Trigger[] leftButtons = new Trigger[2];
  public Trigger[] rightButtons = new Trigger[2];
  public Trigger[] xBoxButtons = new Trigger[10];
  public Trigger[] xBoxPOVButtons = new Trigger[4];
  public Trigger xBoxLeftTrigger, xBoxRightTrigger;

  // The robot's subsystems and commands are defined here...
  
  public static ArmSub mArmSub = new ArmSub();

  public static HandSub mHandSub = new HandSub();

  public static ShoulderSub mShoulderSub = new ShoulderSub();

  public static WristSub mWristSub = new WristSub();

  public static TurnTableSub mTurnTableSub = new TurnTableSub();

  public static PS4Controller operator = new PS4Controller(Constants.operator);

  //Auto Stabalize Button
  JoystickButton Stabalize = new JoystickButton(testController, 5);

  // Turn Hand Buttons 
  JoystickButton HandOpen = new JoystickButton(operator, 7);
  JoystickButton HandClose = new JoystickButton(operator, 8);

  // Hand Buttons
  JoystickButton WristLeft = new JoystickButton(operator, 5);
  JoystickButton WristRight = new JoystickButton(operator, 6);

  // Arm Buttons
  JoystickButton ShoulderUp = new JoystickButton(operator, 4);
  JoystickButton ShoulderDown = new JoystickButton(operator, 2);
  JoystickButton Stowed = new JoystickButton(operator, 12);



/*
 * POV is the D-Pad, Example: POVButton handOpen = new POVButton(operator, 90);
 * The POV buttons are referred to by the angle. Up is 0, right is 90, down is 180, and left is 270.
 * buttonNumber 1 is Square on a PS4 Controller
 * buttonNumber 2 is X on a PS4 Controller
 * buttonNumber 3 is Circle on a PS4 Controller
 * buttonNumber 4 is Triangle on a PS4 Controller
 * buttonNumber 5 is L1 on a PS4 Controller
 * buttonNumber 6 is R1 on a PS4 Controller
 * buttonNumber 7 is L2 on a PS4 Controller
 * buttonNumber 8 is R2 on a PS4 Controller
 * buttonNumber 9 is SHARE on a PS4 Controller
 * buttonNumber 10 is OPTIONS on a PS4 Controller
 * buttonNumber 11 is L3 on a PS4 Controller
 * buttonNumber 12 is R3 on a PS4 Controller
 * buttonNumber 13 is the PlayStaion Button on a PS4 Controller
 * buttonNumber 14 is the Touchpad on a PS4 Controller
 * 
 * https://www.chiefdelphi.com/t/make-motor-move-at-a-specific-rpm/396774
 * Click this link if your trying to send a motor to a certain position
 * 
 * https://docs.wpilib.org/en/stable/docs/software/advanced-controls/controllers/index.html
 * Click this link if your trying to send a motor to a certain position
 * 
 * If you ever run into that problem like The Java Server Crashed 5 Times and will not be restarted, (worked for me)
 * then just delete all the WPILIB VS Codes and The Visual Studio Code that is on the pc, this is because Wpilib uses their own
 * VS Code when you install the latest version!
 */

  public RobotContainer() {
    initializeSubsystems();
    //initializeAutoChooser();

    // Configure the button bindings
    mArmSub.setDefaultCommand(new ArmCommand());
    mTurnTableSub.setDefaultCommand(new TurnTableCommand());
    configureButtonBindings();
   /**  UsbCamera camera = CameraServer.startAutomaticCapture();
    camera.setResolution(160, 120); //Usually (640,320)
    camera.setFPS(30);
    PortForwarder.add(5800, "10.80.6.12", 5800);
    PortForwarder.add(5801, "10.80.6.12", 5801);
    PortForwarder.add(5802, "10.80.6.12", 5802);
  }
*/
  }
  public void initializeSubsystems() {
    m_swerveDrive.setDefaultCommand(
        new SetSwerveDrive(
            m_swerveDrive,
            () -> -testController.getRawAxis(1),
            () -> -testController.getRawAxis(0),
            () -> -testController.getRawAxis(2)));
    m_fieldSim.initSim();
  }

  //Add negative to change direction of the command since this is the axis that appears in the frc drive station.

/**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    for (int i = 0; i < leftButtons.length; i++)
      leftButtons[i] = new JoystickButton(leftJoystick, (i + 1));
    for (int i = 0; i < rightButtons.length; i++)
      rightButtons[i] = new JoystickButton(rightJoystick, (i + 1));
    for (int i = 0; i < xBoxButtons.length; i++)
      xBoxButtons[i] = new JoystickButton(xBoxController, (i + 1));
    for (int i = 0; i < xBoxPOVButtons.length; i++)
      xBoxPOVButtons[i] = new POVButton(xBoxController, (i * 90));

    HandOpen.whileTrue(new OpenHandCommand());
    HandClose.whileTrue(new CloseHandCommand());
    WristLeft.whileTrue(new WristTurnLeftCommand());
    WristRight.whileTrue(new WristTurnRightCommand());
    ShoulderUp.whileTrue(new ShoulderUpCommand());
    ShoulderDown.whileTrue(new ShoulderDownCommand());
    Stowed.onTrue(new Stowed());
    Stabalize.whileTrue(new SetSwerveDriveBalance(m_swerveDrive, null, null, null).withInterruptBehavior(InterruptionBehavior.kCancelIncoming));


    SmartDashboard.putData(new SetSwerveCoastMode(m_swerveDrive));    

  }

 /*  private void initializeAutoChooser() {
    m_autoChooser.setDefaultOption("Do Nothing", new WaitCommand(0));

    m_autoChooser.addOption("curvy path", loadPathplannerTrajectoryToHolonomicCommand(
        "pathplanner/generatedJSON/curvy.wpilib.json",
        true));
        m_autoChooser.addOption("straight", loadPathplannerTrajectoryToHolonomicCommand(
        "pathplanner/generatedJSON/straight.wpilib.json",
        true));

    Shuffleboard.getTab("Autonomous").add(m_autoChooser);

    SmartDashboard.putData("Auto Selector", m_autoChooser);
  }

  public Command loadPathplannerTrajectoryToHolonomicCommand(String filename, boolean resetOdometry) {
    Trajectory trajectory;

    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(filename);
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException exception) {
      DriverStation.reportError("Unable to open trajectory" + filename, exception.getStackTrace());
      System.out.println("Unable to read from file " + filename);
      return new InstantCommand();
    }
  
    HolonomicDriveController controller = new HolonomicDriveController(trajectory, SwerveDrive::getPoseMeters,
        new SimpleMotorFeedforward(Constants.SwerveModule.ksDriveVoltSecondsPerMeter, Constants.SwerveModule.kvDriveVoltSecondsSquaredPerMeter,
            Constants.SwerveModule.kaDriveVoltSecondsSquaredPerMeter),
            Constants.SwerveDive.kSwerveKinematics, SwerveDrive::toSwerveModuleStates,
        new PIDController(Constants.SwerveDrive.kMaxSpeedMetersPerSecond, 0, 0),
        new PIDController(Constants.SwerveDrive.kMaxSpeedMetersPerSecond, 0, 0),
        m_swerveDrive);

    if (resetOdometry) {
      return new SequentialCommandGroup(
          new InstantCommand(() -> SwerveDrive.resetOdometry(trajectory.getInitialPose())), controller);
    } else {
      return controller;
    }

  }
  */
  
    /* 
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoChooser.getSelected();
    //return m_autoCommand;
  }

  public void periodic() {
    m_fieldSim.periodic();
  }

  private void initAutoBuilder() {
    m_eventMap.put("wait", new WaitCommand(5));}

    /*m_autoBuilder =
        new SwerveAutoBuilder(
            m_swerveDrive::getPoseMeters,
            m_swerveDrive::setOdometry,
            Constants.SwerveDrive.kSwerveKinematics,
            m_swerveDrive::setSwerveModuleStates,
            m_eventMap,
            false,
            m_swerveDrive);
  }
  */

  public void disableInit() {
    m_swerveDrive.setIdleMode(IdleMode.kCoast);
  }

  public void teleopInit() {
    m_swerveDrive.setIdleMode(IdleMode.kBrake);
  }


  public void disabledInit() {}

  public void disabledPeriodic() {}

  public void autonomousInit() {}

  public void autonomousPeriodic() {}

  public void teleopPeriodic() {}

  public void simulationInit() {}

  public void simulationPeriodic() {
    REVPhysicsSim.getInstance().run();
  }
}
