// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.driveSubsystem;
import frc.robot.subsystems.flapperSubsystem;
import frc.robot.subsystems.shooterSubsystem;
import frc.robot.subsystems.transitionSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootThenDrive extends SequentialCommandGroup {
  /** Creates a new ShootThenDrive. */
  public ShootThenDrive(
    driveSubsystem drive, shooterSubsystem shoot, transitionSubsystem transition,
    double distance, double speed, double seconds) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      //Shoot preloaded cargo
      new ShootTimeCommand(
        speed, seconds, shoot, transition),
      //Drives to distance
      new DriveDistanceCommand(
        distance, -AutoConstants.kAutoDriveSpeed, drive)
    );
  }
}
