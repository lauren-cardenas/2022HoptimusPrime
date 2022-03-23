// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.TransitionCommand;
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
      new DriveDistanceCommand(distance, robotSpeed, drive),
      new DriveDistanceCommand(-1.45, -robotSpeed, drive)
      .beforeStarting(() -> intake.intakeRun(0.0)),
      new turnSimple(drive, 156, true),
      new ShootTimeCommand(SpeedConstants.aHighCloseShootSpeed, 4, shoot, transition)
      .beforeStarting(() -> intake.intakeRun(SpeedConstants.aRollerSpeed)),
      new turnSimple(drive, 73, true),
      new DriveDistanceCommand(1, -robotSpeed, drive)
     



    );
  }
}
