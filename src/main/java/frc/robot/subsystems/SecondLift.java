// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MechConstants;

public class SecondLift extends SubsystemBase {
  /** Creates a new SecondLift. */
  WPI_TalonFX a_secondlift;
  public SecondLift() {
    a_secondlift = new WPI_TalonFX(MechConstants.aSecondLiftPort);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void secondLift(double speed){
    a_secondlift.set(speed);
  }
}
