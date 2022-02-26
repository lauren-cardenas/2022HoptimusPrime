// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MechConstants;

public class flapperSubsystem extends SubsystemBase {
  /** Creates a new flapperSubsystem. */
  WPI_VictorSPX a_flapper;

  // private DigitalInput flapSwitchDown = new DigitalInput(MechConstants.aFlapDown);
  // private DigitalInput flapSwitchUp = new DigitalInput(MechConstants.aFlapUp);


  public flapperSubsystem() {
    a_flapper = new WPI_VictorSPX(MechConstants.aFlapper);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void flapperRun(double speed){
    a_flapper.set(speed);
  }

  // public boolean getstatusDownFlap(){
  //   return flapSwitchDown.get();
  // }
  // public boolean getstatusUpFlap(){
  //   return flapSwitchUp.get();
  // }

}
