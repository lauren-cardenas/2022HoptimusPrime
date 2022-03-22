// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.nio.file.Path;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.SpeedConstants;
import frc.robot.subsystems.armSubsystem;
import frc.robot.subsystems.driveSubsystem;
import frc.robot.subsystems.flapperSubsystem;
import frc.robot.subsystems.intakeSubsystem;
import frc.robot.subsystems.shooterSubsystem;
import frc.robot.subsystems.transitionSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TwoBallsAuto extends SequentialCommandGroup {
  /** Creates a new ShootThenPathweaver. */

  Trajectory trajectory;
  public TwoBallsAuto(driveSubsystem drivetrain, shooterSubsystem shooter, transitionSubsystem transition,
  armSubsystem arm, intakeSubsystem intake,
  double distance, double speed, double seconds) {

    String myPathName = " ";
    String trajectoryFile = " ";

    myPathName = "output/twoBall";

    trajectoryFile = myPathName + ".wpilib.json";

    try{
      Path pathTrajectory = Filesystem.getDeployDirectory().toPath().resolve(trajectoryFile);
      trajectory = TrajectoryUtil.fromPathweaverJson(pathTrajectory);
  } catch (IOException ex) {
    DriverStation.reportError("Unable to open one or more trajectories",  ex.getStackTrace());
  }

  drivetrain.resetOdometry(trajectory.getInitialPose());
 

  RamseteCommand ramseteCommand =
  new RamseteCommand(
      trajectory,
      drivetrain::getPose,
      new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
      new SimpleMotorFeedforward(
          DriveConstants.asVolts,
          DriveConstants.avVoltSecondsPerMeter,
          DriveConstants.aaVoltSecondsSquaredPerMeter),
      DriveConstants.kDriveKinematics,
      drivetrain::getWheelSpeeds,
      new PIDController(DriveConstants.aPDriveVel, 0, 0),
      new PIDController(DriveConstants.aPDriveVel, 0, 0),
      // RamseteCommand passes volts to the callback
      drivetrain::tankDriveVolts,
      drivetrain);


    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    /*
    addCommands(
      new ShootThenDrive(drivetrain, shooter, transition, distance, speed, seconds, flapper, flapperType, upDown),
      ramseteCommand.andThen(() -> drivetrain.tankDriveVolts(0,0))
    );
    */

    addCommands(
      new ShootThenDrive(drivetrain, shooter, transition, distance, speed, seconds)
      .andThen(() -> drivetrain.resetOdometry(trajectory.getInitialPose())),
      new ArmControlDown(arm)
      .andThen(() -> intake.intakeRun(SpeedConstants.aRollerSpeed)),
      //.andThen(() -> transition.transitionRun(0.3)),
      ramseteCommand.andThen(() -> drivetrain.tankDriveVolts(0,0)),
      new ArmControlUp(arm)
      .andThen(() -> intake.intakeRun(0.0)),
      new ShootTimeCommand(0.39, seconds, shooter, transition)
    );
    
  }
}


  