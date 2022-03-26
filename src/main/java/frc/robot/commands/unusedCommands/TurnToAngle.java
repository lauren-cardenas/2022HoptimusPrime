// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.unusedCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.driveSubsystem;

public class TurnToAngle extends CommandBase {
  /** Creates a new TurnToAngle. */
  private double m_angle;
  private double m_targetAngle;
  private final double lowSpeed = 0.2;
  private final double highSpeed = 0.4;
  private final double closeAngle = 10;
  private final driveSubsystem m_drive;

  public TurnToAngle(double angle, driveSubsystem drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = drive;
    addRequirements(drive);
    m_angle = angle;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double heading = m_drive.getHeading();
      m_targetAngle = heading + m_angle;
    
    double difference = (m_targetAngle - heading) % 360.0;
    if (difference < -180){
      difference += 360;
    }
    if(difference > 180){
      difference -= 360;
    }

    double differenceAbs = Math.abs(difference);
    double differenceSign = Math.signum(difference);

    if (differenceAbs > closeAngle) {
      m_drive.tankDrive(differenceSign * highSpeed, differenceSign * -highSpeed);
    } else {
      m_drive.tankDrive(differenceSign * lowSpeed, differenceSign * -lowSpeed);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.tankDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double heading = m_drive.getHeading();
    double difference = (m_targetAngle - heading) % 360.0;
    if(difference < -180){
      difference += 360;
    } 
    if(difference > 180){
      difference -= 360;
    }

    if (Math.abs(difference) <= 15){
      return true;
    }
      return false;

  }
}
