// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.driveSubsystem;
import frc.robot.subsystems.transitionSubsystem;

public class turnSimple extends CommandBase {
  /** Creates a new turnSimple. */
  private final driveSubsystem drive;
  private final int kAngle;
  private final boolean kDirection;
  private final transitionSubsystem transition;

  public turnSimple(driveSubsystem m_drive, int angle, boolean direction, transitionSubsystem a_transition) {
    // Use addRequirements() here to declare subsystem dependencies.
    drive = m_drive;
    kAngle = angle;
    kDirection = direction;
    transition = a_transition;
    addRequirements(drive, transition);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drive.zeroHeading();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(kDirection){
      drive.arcadeDrive(0.0, AutoConstants.kAutoTurnSpeed);
    } else{
      drive.arcadeDrive(0.0, -AutoConstants.kAutoTurnSpeed);
    }
    transition.transitionRun(0.0);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.arcadeDrive(0.0, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(kDirection){
      return drive.getHeading() <= -kAngle;
    } else{
      return drive.getHeading() >= -kAngle;
    }
  }
}
