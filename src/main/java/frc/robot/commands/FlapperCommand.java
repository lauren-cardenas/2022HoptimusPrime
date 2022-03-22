// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.Constants.SpeedConstants;
// import frc.robot.subsystems.flapperSubsystem;

// public class FlapperCommand extends CommandBase {
//   /** Creates a new FlapperCommand. */
//   private final flapperSubsystem flapper;
//   private final boolean m_direction;
//   private final int m_speed;

//   public FlapperCommand(boolean direction, int speed, flapperSubsystem flapperSub) {
//     // Use addRequirements() here to declare subsystem dependencies.
//     flapper = flapperSub;
//     m_direction = direction;
//     m_speed = speed;
//     addRequirements(flapper);
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
    
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     flapper.flapperRun(m_speed * SpeedConstants.aFlabberSpeed);
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//     flapper.flapperRun(0.0);
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     if(m_direction == false){
//       return flapper.getstatusUpFlap() == false;
//     } else {
//       return flapper.getstatusDownFlap() == false;
//     }
//   }
// }
