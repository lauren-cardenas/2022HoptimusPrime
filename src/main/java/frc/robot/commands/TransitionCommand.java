// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.transitionSubsystem;
import frc.robot.Constants.SpeedConstants;

public class TransitionCommand extends CommandBase {
  private final transitionSubsystem m_transition;
  /** Creates a new TransitionCommand. */
  public TransitionCommand(transitionSubsystem transitionsub) {
    m_transition = transitionsub;
    addRequirements(m_transition);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_transition.transitionRun(-SpeedConstants.aTransitionSpeedAuto);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_transition.transitionRun(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
