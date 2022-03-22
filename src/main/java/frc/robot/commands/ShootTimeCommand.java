// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.SpeedConstants;
import frc.robot.subsystems.shooterSubsystem;
import frc.robot.subsystems.transitionSubsystem;

public class ShootTimeCommand extends CommandBase {
  /** Creates a new ShootTimeCommand. */
  private final shooterSubsystem m_shoot;
  private final transitionSubsystem m_transition;
  private final double m_shootSpeed;
  private final double m_seconds;


  private final Timer m_timer = new Timer();

  public ShootTimeCommand(
    double shootSpeed, double seconds,
    shooterSubsystem shoot, transitionSubsystem transition) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shootSpeed = shootSpeed;
    m_seconds = seconds;
    m_shoot = shoot;
    m_transition = transition;
    addRequirements(m_shoot, m_transition);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.reset();
    m_timer.start();
    
    m_shoot.shooterRun(m_shootSpeed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_shoot.shooterRun(m_shootSpeed);
    if (m_timer.get() > 2.0){
      m_transition.transitionRun(SpeedConstants.aTransitionSpeedAuto-0.06);

    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shoot.shooterRun(0.0);
    m_transition.transitionRun(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_timer.get() >= m_seconds;
  }
}
