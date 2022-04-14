// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriverButtons;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.OperatorButtons;
import frc.robot.Constants.SpeedConstants;
import frc.robot.commands.ArmControlDown;
import frc.robot.commands.ArmControlUp;
import frc.robot.commands.AutoThreeBall;
import frc.robot.commands.AutoTwoBall;
import frc.robot.commands.DriveTimeCommand;
import frc.robot.commands.RAutoTwoBall;
import frc.robot.commands.ShootThenDrive;
import frc.robot.commands.ThreeBallPath;
import frc.robot.commands.beastMode;
import frc.robot.commands.unusedCommands.AutoTwoBallWall;
import frc.robot.subsystems.armSubsystem;
import frc.robot.subsystems.driveSubsystem;
import frc.robot.subsystems.intakeSubsystem;
import frc.robot.subsystems.liftSubsystem;
import frc.robot.subsystems.shooterSubsystem;
import frc.robot.subsystems.transitionSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final driveSubsystem a_robotDrive = new driveSubsystem();
  private final shooterSubsystem a_shooter = new shooterSubsystem();
  private final intakeSubsystem a_rollerIntake = new intakeSubsystem();
  private final armSubsystem a_arm = new armSubsystem();
  private final transitionSubsystem a_transition = new transitionSubsystem();
  private final liftSubsystem a_lift = new liftSubsystem();

  XboxController a_driverController = new XboxController(OIConstants.aDriverControllerPort);
  XboxController a_operatorController = new XboxController(OIConstants.aOperatorControllerPort);

  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  private final Command m_simpleAuto =
    new DriveTimeCommand(
      AutoConstants.kAutoDriveTime, AutoConstants.kAutoDriveSpeed, a_robotDrive);
  
  private final Command m_ShootCloseDrive =
    new ShootThenDrive(a_robotDrive, a_shooter, a_transition, 1.5, SpeedConstants.aAutoOneBallShootSpeed,AutoConstants.transitionTime);
    
  private final Command a_ShootLowThenDrive = 
    new ShootThenDrive(a_robotDrive, a_shooter, a_transition, AutoConstants.shootLowDistance, SpeedConstants.aHigherShootSpeed, AutoConstants.transitionTime );

  private final Command m_twoBallAuto =
    new AutoTwoBall(1.5, AutoConstants.kAutoDriveSpeed, a_robotDrive, a_arm, a_rollerIntake, a_transition, a_shooter);

  private final Command m_twoBallWallAuto =
    new AutoTwoBallWall(1.5, AutoConstants.kAutoDriveSpeed, a_robotDrive, a_arm, a_rollerIntake, a_transition, a_shooter);  
  
  private final Command m_threeBallAuto = 
    new AutoThreeBall(1.5, AutoConstants.kAutoDriveSpeed - 0.05, a_robotDrive, a_arm, a_rollerIntake, a_transition, a_shooter);
  private final Command m_beastMode = 
    new beastMode(5.5, -.6, a_robotDrive, a_arm, a_rollerIntake, a_transition, a_shooter);

  private final Command m_PATHthreeBall =
    new ThreeBallPath(a_robotDrive, a_shooter, a_transition, a_arm, a_rollerIntake);

  private final Command m_RtwoBallAuto = 
    new RAutoTwoBall(1.5, AutoConstants.kAutoDriveSpeed, a_robotDrive, a_arm, a_rollerIntake, a_transition, a_shooter);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
  
    // Camera related stuff
   UsbCamera camera = CameraServer.startAutomaticCapture();
   camera.setResolution(80, 45);
   camera.setFPS(30);
   

    // Configure the button bindings
    configureButtonBindings();

    // Driver Controller Driving
    /*
    a_robotDrive.setDefaultCommand(
      new RunCommand(() -> a_robotDrive.arcadeDrive(
        (a_driverController.getRightTriggerAxis()
           - a_driverController.getLeftTriggerAxis())* SpeedConstants.driveSpeed,
        a_driverController.getLeftX() * SpeedConstants.MturnSpeed
      ), a_robotDrive));
    */

      // Operator Controller Driving 
      a_robotDrive.setDefaultCommand(
        new RunCommand(() -> a_robotDrive.arcadeDrive(
          (a_operatorController.getRightTriggerAxis()
             - a_operatorController.getLeftTriggerAxis()),//* SpeedConstants.driveSpeed,
          a_operatorController.getLeftX() * SpeedConstants.MturnSpeed
        ), a_robotDrive));

      
  
      // Must be there
      //SmartDashboard.putData(a_robotDrive);
      //a_robotDrive.displayEncoderValues();
      //SmartDashboard.putData(a_shooter);

    //Auto Choices
    autoChooser.setDefaultOption("One Ball",m_ShootCloseDrive);
    autoChooser.addOption("Shoot Low", a_ShootLowThenDrive);
    autoChooser.addOption("Simple Auto", m_simpleAuto);
    autoChooser.addOption("Two Ball", m_twoBallAuto);
    autoChooser.addOption("Right Two Ball", m_RtwoBallAuto);

    //Shuffleboard.getTab("Autonomous").add(autoChooser);
    SmartDashboard.putData("Autonomous", autoChooser);

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   * @param Button 
   */
  private void configureButtonBindings() {

    //*************driver****************//    
    a_transition.setDefaultCommand(
      new RunCommand(() -> a_transition.transitionRun((-a_driverController.getRightY()+(-a_operatorController.getRightY()))*SpeedConstants.aTransitionSpeed),a_transition));

      // 
      new JoystickButton(a_driverController, DriverButtons.bLiftRun)
      .whenPressed(() -> a_lift.liftRun(-SpeedConstants.aLiftSpeed))
      .whenReleased(() -> a_lift.liftRun(0.0));

      new JoystickButton(a_driverController, DriverButtons.bLiftback)
      .whenPressed(() -> a_lift.liftRun(SpeedConstants.aLiftSpeed))
      .whenReleased(() -> a_lift.liftRun(0.0));

      new JoystickButton(a_driverController, DriverButtons.bWinchRun)
      .whenPressed(() -> a_lift.winchRun(-SpeedConstants.aWinchSpeed))
      .whenReleased(() -> a_lift.winchRun(0.0));

      new JoystickButton(a_driverController, DriverButtons.bWinchBack)
      .whenPressed(() -> a_lift.winchRun(SpeedConstants.aWinchSpeed))
      .whenReleased(() -> a_lift.winchRun(0.0));

    
    //Shooter (High)
    new POVButton(a_driverController, 0)
    .whenPressed(() -> a_shooter.shooterRun(SpeedConstants.aHighShootSpeed))
    .whenReleased(() -> a_shooter.shooterRun(0.0));//changed

    //Shooter (Low)
    new POVButton(a_driverController, 180)
    .whenPressed(() -> a_shooter.shooterRun(SpeedConstants.aHigherShootSpeed))
    .whenReleased(() -> a_shooter.shooterRun(0.0));//changed

    // new POVButton(a_driverController, 90)
    // .whenPressed(() -> a_shooter.shooterRun(0.5))
    // .whenReleased(() -> a_shooter.shooterRun(0.0));
    

    //*************operator****************//

    

    new JoystickButton(a_operatorController, OperatorButtons.bIntakeRun)
    .whenPressed(() -> a_rollerIntake.intakeRun(SpeedConstants.aRollerSpeed))
    .whenReleased(() -> a_rollerIntake.intakeRun(0.0));

    new JoystickButton(a_operatorController, OperatorButtons.bArmUp)
    .whenPressed(new ArmControlUp(a_arm))
    .whenReleased(() -> a_arm.intakeArmStop());

    new JoystickButton(a_operatorController, OperatorButtons.bArmDown)
    .whenPressed(new ArmControlDown(a_arm))
    .whenReleased(() -> a_arm.intakeArmStop());

    new JoystickButton(a_operatorController, OperatorButtons.bHalfSpeed)
     .whenPressed(() -> a_robotDrive.setMaxOutput(0.5))
     .whenReleased(() -> a_robotDrive.setMaxOutput(0.9));

     new JoystickButton(a_operatorController, OperatorButtons.bFullSpeed)
     .whenPressed(() -> a_robotDrive.setMaxOutput(1.0))
     .whenReleased(() -> a_robotDrive.setMaxOutput(0.9));

    new JoystickButton(a_operatorController, OperatorButtons.bTurn)
    .whenPressed(() -> a_rollerIntake.intakeRun(-SpeedConstants.aRollerSpeed))
    .whenReleased(() -> a_rollerIntake.intakeRun(0.0));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Get autonomous command from chooser
    return autoChooser.getSelected();
  }
}


            














