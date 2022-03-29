// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.SpeedConstants;
import frc.robot.subsystems.armSubsystem;
import frc.robot.subsystems.driveSubsystem;
import frc.robot.subsystems.intakeSubsystem;
import frc.robot.subsystems.shooterSubsystem;
import frc.robot.subsystems.transitionSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoTwoBall extends SequentialCommandGroup {
  /** Creates a new AutoTwoBall. */
  public AutoTwoBall(double distance, double robotSpeed, 
  driveSubsystem drive, armSubsystem arm, intakeSubsystem intake, transitionSubsystem transition, shooterSubsystem shoot) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ArmControlDown(arm)
      .beforeStarting(() -> intake.intakeRun(SpeedConstants.aRollerSpeed)),
      new DriveDistanceCommand(distance, robotSpeed, drive),
      new DriveDistanceCommand(-1.45, -robotSpeed, drive)
      .beforeStarting(() -> intake.intakeRun(0.0)),
      new turnSimple(drive, 156, true)
      .beforeStarting(() -> shoot.shooterRun(SpeedConstants.aHighShootSpeed)),
      new RunCommand(() -> transition.transitionRun(SpeedConstants.aTransitionSpeedAuto))
      //new ShootTimeCommand(SpeedConstants.aHighCloseShootSpeed, 0.25, shoot, transition)
      .beforeStarting(() -> intake.intakeRun(SpeedConstants.aRollerSpeed))
      .raceWith(new WaitCommand(3)),
      new DriveDistanceCommand(-1.5, -robotSpeed, drive)
      .beforeStarting(new ShootTimeCommand(0, 0, shoot, transition))
      .beforeStarting(()-> intake.intakeRun(0.0))
      .beforeStarting(() -> transition.transitionRun(0.0))
    );
  }
}
