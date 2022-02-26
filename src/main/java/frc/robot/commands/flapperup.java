// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.SpeedConstants;
import frc.robot.subsystems.flapperSubsystem;

public class flapperup extends CommandBase {
  /** Creates a new flapperup. */
  private final flapperSubsystem M_flapperUp;
  public flapperup(flapperSubsystem flappersub) {
    M_flapperUp = flappersub;
    addRequirements(M_flapperUp);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    M_flapperUp.flapperRun(SpeedConstants.aFlabberSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    M_flapperUp.flapperRun(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return M_flapperUp.getstatusUpFlap() == false;
  }
}
