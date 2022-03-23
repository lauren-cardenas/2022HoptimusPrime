// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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
  public AutoTwoBall(double distance, double speed, 
  driveSubsystem drive, armSubsystem arm, intakeSubsystem intake, transitionSubsystem transition, shooterSubsystem shoot) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ArmControlDown(arm)
      .beforeStarting(() -> intake.intakeRun(SpeedConstants.aRollerSpeed)),
      new DriveDistanceCommand(distance, speed, drive)
      .beforeStarting(() -> transition.transitionRun(SpeedConstants.aTransitionSpeedAuto)),
      new DriveDistanceCommand(-distance, -speed, drive),
      new GyroTurnTo(180, drive),
      new ShootTimeCommand(SpeedConstants.aHighCloseShootSpeed, 4, shoot, transition)
    );
  }
}
