// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.armSubsystem;
//import frc.robot.subsystems.intakeSubsystem;

public class ArmControlDown extends CommandBase {
  /** Creates a new ArmControlDown. */
  private final armSubsystem a_armDown;

  public ArmControlDown(armSubsystem armSub) {
    // Use addRequirements() here to declare subsystem dependencies.
    a_armDown = armSub;
    addRequirements(a_armDown);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    a_armDown.intakeDown();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    a_armDown.intakeArmStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return a_armDown.getstatusDown() == false;
  }
}
