// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.unusedCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.SpeedConstants;
import frc.robot.commands.ArmControlDown;
import frc.robot.commands.DriveDistanceCommand;
import frc.robot.subsystems.armSubsystem;
import frc.robot.subsystems.driveSubsystem;
import frc.robot.subsystems.intakeSubsystem;
import frc.robot.subsystems.shooterSubsystem;
import frc.robot.subsystems.transitionSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class beastMode extends SequentialCommandGroup {
 
  public beastMode(double distance, double robotSpeed,
  driveSubsystem drive, armSubsystem arm, intakeSubsystem intake,
  transitionSubsystem transition, shooterSubsystem shoot
 ) {  
    
  
  addCommands(

    //second phase BEAST MODE
    new ArmControlDown(arm)
    .beforeStarting(() -> intake.intakeRun(SpeedConstants.aRollerSpeed)),
    new DriveDistanceCommand(distance, robotSpeed, drive),
    new DriveDistanceCommand(-distance, -robotSpeed, drive)
  



    );
  }
}
