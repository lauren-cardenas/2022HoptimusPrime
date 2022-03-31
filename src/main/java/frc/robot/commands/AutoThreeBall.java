// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.SpeedConstants;
import frc.robot.subsystems.armSubsystem;
import frc.robot.subsystems. driveSubsystem;
import frc.robot.subsystems.intakeSubsystem;
import frc.robot.subsystems.transitionSubsystem;
import frc.robot.subsystems.shooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoThreeBall extends SequentialCommandGroup {
  /** Creates a new AutoThreeBall. */
  public AutoThreeBall(double distance, double robotSpeed, 
  driveSubsystem drive, armSubsystem arm, intakeSubsystem intake, transitionSubsystem transition, shooterSubsystem shoot) {
    
    addCommands(
      //new ArmControlDown(arm)
      new DriveDistanceCommand(0.8, robotSpeed - 0.15, drive)
      .alongWith(new ArmControlDown(arm))
      .beforeStarting(() -> intake.intakeRun(SpeedConstants.aRollerSpeed)),
       //go to second ball
      new DriveDistanceCommand(-0.8, -robotSpeed, drive) //backup to shoot
      .beforeStarting(() -> intake.intakeRun(0.4)),
      new turnSimple(drive, -155, false, transition) //turn to goal
      .beforeStarting(() -> drive.zeroHeading())
      .beforeStarting(() -> shoot.shooterRun(SpeedConstants.aHighShootSpeed)),
      new RunCommand(() -> transition.transitionRun(SpeedConstants.aTransitionSpeedAuto))
      .raceWith(new WaitCommand(1.5)),
     // new RunCommand(() -> transition.transitionRun(0.0)),
      new turnSimple(drive, -93, false, transition)
      .beforeStarting(() -> shoot.shooterRun(0.0)), //turn to third ball
      new DriveDistanceCommand(1.55, robotSpeed - 0.15, drive),
      new turnSimple(drive, 140, true, transition)
      .beforeStarting(() -> shoot.shooterRun(0.45)), //turn to goal
      new RunCommand(() -> transition.transitionRun(SpeedConstants.aTransitionSpeedAuto))
      .raceWith(new WaitCommand(1.5)),
      new RunCommand(() -> shoot.shooterRun(0), shoot)
      .beforeStarting(new ShootTimeCommand(0, 0, shoot, transition))
      .beforeStarting(()-> intake.intakeRun(0.0))
      .beforeStarting(() -> transition.transitionRun(0.0)),
      new DriveDistanceCommand(0.0, -robotSpeed, drive)
      .beforeStarting(new ShootTimeCommand(0, 0, shoot, transition))
      .beforeStarting(()-> intake.intakeRun(0.0))
      .beforeStarting(() -> transition.transitionRun(0.0))
     



    );
  }
}
