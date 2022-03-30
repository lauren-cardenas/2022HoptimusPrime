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
      new ArmControlDown(arm)
      .beforeStarting(() -> intake.intakeRun(SpeedConstants.aRollerSpeed)),
      new DriveDistanceCommand(0.8, robotSpeed - 0.15, drive), //go to second ball
      new DriveDistanceCommand(-0.8, -robotSpeed, drive) //backup to shoot
      .beforeStarting(() -> intake.intakeRun(0.4)),
      new turnSimple(drive, 200, true) //turn to goal
      .beforeStarting(() -> drive.zeroHeading())
      .beforeStarting(() -> shoot.shooterRun(SpeedConstants.aHighShootSpeed)),
      new RunCommand(() -> transition.transitionRun(SpeedConstants.aTransitionSpeedAuto))
      //.beforeStarting(() -> intake.intakeRun(SpeedConstants.aRollerSpeed))
      .raceWith(new WaitCommand(2.5)),
      new turnSimple(drive, 90, false), //turn to third ball
      new DriveDistanceCommand(1, robotSpeed - 0.15, drive),
      new turnSimple(drive, 290, true), //turn to goal
      new RunCommand(() -> transition.transitionRun(SpeedConstants.aTransitionSpeedAuto))
      .raceWith(new WaitCommand(1)),
      new RunCommand(() -> shoot.shooterRun(0), shoot)
      .beforeStarting(()-> intake.intakeRun(0.0))
      .beforeStarting(() -> transition.transitionRun(0.0))
     



    );
  }
}
