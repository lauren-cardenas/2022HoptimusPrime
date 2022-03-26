// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
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
import frc.robot.commands.ShootThenDrive;
import frc.robot.commands.TurnToAngle;
import frc.robot.commands.TwoBallsAuto;
<<<<<<< HEAD
import frc.robot.commands.turnSimple;
=======
// import frc.robot.commands.flapperdown;
// import frc.robot.commands.flapperup;
>>>>>>> a30fa14e00b1f00dcdc02f02e05e77863d9cbc3f
import frc.robot.subsystems.SecondLift;
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
  private final SecondLift a_sSecondLift = new SecondLift();

  XboxController a_driverController = new XboxController(OIConstants.aDriverControllerPort);
  XboxController a_operatorController = new XboxController(OIConstants.aOperatorControllerPort);

  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  private final Command m_simpleAuto =
    new DriveTimeCommand(
      AutoConstants.kAutoDriveTime, AutoConstants.kAutoDriveSpeed, a_robotDrive);

  private final Command m_ShootThenDrive = 
    new ShootThenDrive(a_robotDrive, a_shooter, a_transition, AutoConstants.shootHighDistance, SpeedConstants.aHighShootSpeed,AutoConstants.transitionTime);
  
  private final Command m_ShootCloseDrive =
    new ShootThenDrive(a_robotDrive, a_shooter, a_transition, AutoConstants.shootHighDistance, SpeedConstants.aHighCloseShootSpeed,AutoConstants.transitionTime);
    
  private final Command a_ShootLowThenDrive = 
    new ShootThenDrive(a_robotDrive, a_shooter, a_transition, AutoConstants.shootLowDistance, SpeedConstants.aLowShootSpeed, AutoConstants.transitionTime );

  private final Command m_twoBallAuto =
    new AutoTwoBall(1.5, AutoConstants.kAutoDriveSpeed, a_robotDrive, a_arm, a_rollerIntake, a_transition, a_shooter);
  
  private final Command m_threeBallAuto = 
    new AutoThreeBall(1.5, AutoConstants.kAutoDriveSpeed, a_robotDrive, a_arm, a_rollerIntake, a_transition, a_shooter); 

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
   CameraServer.startAutomaticCapture();

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
      SmartDashboard.putData(a_robotDrive);
      a_robotDrive.displayEncoderValues();

    //Auto Choices
    autoChooser.setDefaultOption("Simple Auto", m_simpleAuto);
    autoChooser.addOption("shoot then drive", m_ShootThenDrive);
    autoChooser.addOption("Shoot Low", a_ShootLowThenDrive);
    autoChooser.addOption("Shoot close high",m_ShootCloseDrive);
    autoChooser.addOption("Two Ball", m_twoBallAuto);
    autoChooser.addOption("Three Ball", m_threeBallAuto);


    autoChooser.addOption("ShootThenPathweaver", new TwoBallsAuto(a_robotDrive, a_shooter, a_transition, a_arm, a_rollerIntake, AutoConstants.shootHighDistance, SpeedConstants.aHighShootSpeed, AutoConstants.transitionTime));
    //autoChooser.addOption("Three Balls", getPathweaverCommand(0));
    // autoChooser.addOption("Two Ball", getPathweaverCommand(1));
    //autoChooser.addOption("Straight", getPathweaverCommand(0));

    Shuffleboard.getTab("Autonomous").add(autoChooser);

    // Changing Path weaver 
    // autoChooser.addOption("M&A_PathWeaver Test",getPathweaverCommand(0));

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

      new JoystickButton(a_driverController, DriverButtons.bSecondLiftUp)
      .whenPressed(() -> a_sSecondLift.secondLift(SpeedConstants.aSecondLiftSpeed))
      .whenReleased(() -> a_sSecondLift.secondLift(0.0));

      new JoystickButton(a_driverController, DriverButtons.bSecondLiftDown)
      .whenPressed(() -> a_sSecondLift.secondLift(-SpeedConstants.aSecondLiftSpeed))
      .whenReleased(() -> a_sSecondLift.secondLift(0.0));
    

    //*************operator****************//

    //Shooter (High)
    new POVButton(a_operatorController, 0)
    .whenPressed(() -> a_shooter.shooterRun(SpeedConstants.aHighShootSpeed))
    .whenReleased(() -> a_shooter.shooterRun(0.0));//changed

    //Shooter (Low)
    new POVButton(a_operatorController, 180)
    .whenPressed(() -> a_shooter.shooterRun(SpeedConstants.aLowShootSpeed))
    .whenReleased(() -> a_shooter.shooterRun(0.0));//changed

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
    .whenPressed(new turnSimple(a_robotDrive, 90, true));

  }
//   public Command getPathweaverCommand(int json){

//     String[] trajectoryJSON =
//     {//"output/threeBall.wpilib.json",
//       //"output/sPath.wpilib.json",
//       "output/Straight.wpilib.json",
//       "output/twoBall.wpilib.json"
//     };
// /*
//     // Create a voltage constraint to ensure we don't accelerate too fast
//     var autoVoltageConstraint =
//         new DifferentialDriveVoltageConstraint(
//             new SimpleMotorFeedforward(
//                 DriveConstants.asVolts,
//                 DriveConstants.avVoltSecondsPerMeter,
//                 DriveConstants.aaVoltSecondsSquaredPerMeter),
//             DriveConstants.kDriveKinematics,
//             10);

//     // Create config for trajectory
//     TrajectoryConfig config =
//         new TrajectoryConfig(
//                 AutoConstants.kMaxSpeedMetersPerSecond,
//                 AutoConstants.kMaxAccelerationMetersPerSecondSquared)
//             // Add kinematics to ensure max speed is actually obeyed
//             .setKinematics(DriveConstants.kDriveKinematics)
//             // Apply the voltage constraint
//             .addConstraint(autoVoltageConstraint);
// */

//     Trajectory trajectory = new Trajectory();

//     try{
//         Path pathTrajectory = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON[json]);
//         trajectory = TrajectoryUtil.fromPathweaverJson(pathTrajectory);
//     } catch (IOException ex) {
//       DriverStation.reportError("Unable to open one or more trajectories",  ex.getStackTrace());
//     }

//     RamseteCommand ramseteCommand =
//         new RamseteCommand(
//             trajectory,
//             a_robotDrive::getPose,
//             new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
//             new SimpleMotorFeedforward(
//                 DriveConstants.asVolts,
//                 DriveConstants.avVoltSecondsPerMeter,
//                 DriveConstants.aaVoltSecondsSquaredPerMeter),
//             DriveConstants.kDriveKinematics,
//             a_robotDrive::getWheelSpeeds,
//             new PIDController(DriveConstants.aPDriveVel, 0, 0),
//             new PIDController(DriveConstants.aPDriveVel, 0, 0),
//             // RamseteCommand passes volts to the callback
//             a_robotDrive::tankDriveVolts,
//             a_robotDrive);

//     //reset odometry
//     a_robotDrive.resetOdometry(trajectory.getInitialPose());

//     return ramseteCommand.andThen(() -> a_robotDrive.tankDriveVolts(0, 0));
//   }

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


            














