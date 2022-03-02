// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MechConstants;

public class liftSubsystem extends SubsystemBase {
  /** Creates a new liftSubsystem. */
  WPI_VictorSPX a_lift;
  WPI_VictorSPX a_winch;

  private final DigitalInput liftSwitch = new DigitalInput(MechConstants.aLiftSwitchPort);

  public liftSubsystem() {
    a_lift = new WPI_VictorSPX(MechConstants.aLiftPort);
    a_winch = new WPI_VictorSPX(MechConstants.aWinchPort);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void liftRun(double speed){
    a_lift.set(-speed);
  }

  public void winchRun(double speed){
    a_winch.set(speed);
  }
    public boolean getstatusLift(){
    return liftSwitch.get();
  }
}
