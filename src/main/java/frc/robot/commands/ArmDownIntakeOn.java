// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.SpeedConstants;
import frc.robot.subsystems.armSubsystem;
import frc.robot.subsystems.intakeSubsystem;

public class ArmDownIntakeOn extends CommandBase {
  /** Creates a new ArmDownIntakeOn. */
  private final armSubsystem a_armDown;
  private final intakeSubsystem a_intake;
  public ArmDownIntakeOn(armSubsystem armSub, intakeSubsystem intakeSub) {
    // Use addRequirements() here to declare subsystem dependencies.
    a_armDown = armSub;
    a_intake = intakeSub;
    addRequirements(a_armDown, a_intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    new ArmControlDown(a_armDown);
    a_intake.intakeRun(-SpeedConstants.aRollerSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return a_armDown.getstatusDown() == true;
  }
}
