// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.DriverButtons;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.SpeedConstants;
import frc.robot.commands.ArmControlDown;
import frc.robot.commands.ArmControlUp;
//import frc.robot.commands.ArmDownIntakeOn;
import frc.robot.commands.DriveTimeCommand;
import frc.robot.subsystems.armSubsystem;
import frc.robot.subsystems.driveSubsystem;
import frc.robot.subsystems.flapperSubsystem;
import frc.robot.subsystems.intakeSubsystem;
import frc.robot.subsystems.liftSubsystem;
import frc.robot.subsystems.shooterSubsystem;
import frc.robot.subsystems.transitionSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

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
  private final flapperSubsystem a_flabber = new flapperSubsystem();
  private final liftSubsystem a_lift = new liftSubsystem();

  XboxController a_driverController = new XboxController(OIConstants.aDriverControllerPort);
  XboxController a_operatorController = new XboxController(OIConstants.aOperatorControllerPort);

  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  private final Command m_simpleAuto =
    new DriveTimeCommand(
      AutoConstants.kAutoDriveTime, AutoConstants.kAutoDriveSpeed, a_robotDrive);


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
   CameraServer.startAutomaticCapture();

    // Configure the button bindings
    configureButtonBindings();

    a_robotDrive.setDefaultCommand(
      new RunCommand(() -> a_robotDrive.arcadeDrive(
        (a_driverController.getRightTriggerAxis()
           - a_driverController.getLeftTriggerAxis())* SpeedConstants.driveSpeed,
        a_driverController.getLeftX() * SpeedConstants.turnSpeed
      ), a_robotDrive));
  
      // Must be there
      SmartDashboard.putData(a_robotDrive);
      a_robotDrive.displayEncoderValues();

    //Auto Choices
    autoChooser.setDefaultOption("Simple Auto", m_simpleAuto);

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    //*************driver****************//

    new JoystickButton(a_driverController, DriverButtons.bShooterRun)
    .whenPressed(() -> a_shooter.shooterRun(SpeedConstants.aHighShootSpeed))
    .whenReleased(() -> a_shooter.shooterRun(0.0));
    
    a_transition.setDefaultCommand(
      new RunCommand(() -> a_transition.transitionRun(-a_driverController.getRightY()),a_transition));

    new JoystickButton(a_driverController, DriverButtons.bIntakeRun)
    .whenPressed(() -> a_rollerIntake.intakeRun(SpeedConstants.aRollerSpeed))
    .whenReleased(() -> a_rollerIntake.intakeRun(0.0));

    new JoystickButton(a_driverController, DriverButtons.bArmUp)
    .whenPressed(new ArmControlUp(a_arm))
    .whenReleased(() -> a_arm.intakeArmStop());

    new JoystickButton(a_driverController, DriverButtons.bArmDown)
    .whenPressed(new ArmControlDown(a_arm))
    .whenReleased(() -> a_arm.intakeArmStop());

    // new JoystickButton(a_driverController, DriverButtons.bTransitionRun)
    // .whenPressed(() -> a_transition.transitionRun(SpeedConstants.aTransitionSpeed))
    // .whenReleased(() -> a_transition.transitionStop());

    // new JoystickButton(a_driverController, DriverButtons.bFlapperRun)
    // .whenPressed(() -> a_flabber.flapperRun(SpeedConstants.aFlabberSpeed))
    // .whenReleased(() -> a_flabber.flapperRun(0.0));

    //*************operator****************//
    new JoystickButton(a_operatorController, DriverButtons.bShooterRunOperator)
    .whenPressed(() -> a_shooter.shooterRun(SpeedConstants.aHighShootSpeed))
    .whenReleased(() -> a_shooter.shooterRun(0.0));

    new JoystickButton(a_operatorController, DriverButtons.bIntakeRun)
    .whenPressed(() -> a_rollerIntake.intakeRun(SpeedConstants.aRollerSpeed))
    .whenReleased(() -> a_rollerIntake.intakeRun(0.0));

    new JoystickButton(a_operatorController, DriverButtons.bArmUp)
    .whenPressed(new ArmControlUp(a_arm))
    .whenReleased(() -> a_arm.intakeArmStop());

    new JoystickButton(a_operatorController, DriverButtons.bArmDown)
    .whenPressed(new ArmControlDown(a_arm))
    .whenReleased(() -> a_arm.intakeArmStop());

    new JoystickButton(a_operatorController, DriverButtons.bLiftRun)
    .whenPressed(() -> a_lift.liftRun(-SpeedConstants.aLiftSpeed))
    .whenReleased(() -> a_lift.liftRun(0.0));

    new JoystickButton(a_operatorController, DriverButtons.bWinchRun)
    .whenPressed(() -> a_lift.winchRun(-SpeedConstants.aWinchSpeed))
    .whenReleased(() -> a_lift.winchRun(0.0));

    a_transition.setDefaultCommand(
      new RunCommand(() -> a_transition.transitionRun(-a_driverController.getRightY()),a_transition));
  }
  public Command getPathweaverCommand(int json){

    String[] trajectoryJSON =
    {"threeBall.wpilib.json",
    ""};

    Trajectory trajectory = new Trajectory();

    try{
        Path pathTrajectory = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON[json]);
        trajectory = TrajectoryUtil.fromPathweaverJson(pathTrajectory);
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open one or more trajectories",  ex.getStackTrace());
    }

    RamseteCommand ramseteCommand =
        new RamseteCommand(
            trajectory,
            a_robotDrive::getPose,
            new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
            new SimpleMotorFeedforward(
                DriveConstants.asVolts,
                DriveConstants.avVoltSecondsPerMeter,
                DriveConstants.aaVoltSecondsSquaredPerMeter),
            DriveConstants.kDriveKinematics,
            a_robotDrive::getWheelSpeeds,
            new PIDController(DriveConstants.aPDriveVel, 0, 0),
            new PIDController(DriveConstants.aPDriveVel, 0, 0),
            // RamseteCommand passes volts to the callback
            a_robotDrive::tankDriveVolts,
            a_robotDrive);

    //reset odometry
    a_robotDrive.resetOdometry(trajectory.getInitialPose());

    return ramseteCommand.andThen(() -> a_robotDrive.tankDriveVolts(0, 0));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return autoChooser.getSelected();
  }
}
