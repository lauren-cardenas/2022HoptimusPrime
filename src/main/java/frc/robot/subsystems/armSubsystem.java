// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MechConstants;
import frc.robot.Constants.SpeedConstants;

public class armSubsystem extends SubsystemBase {
  /** Creates a new armSubsystem. */
  WPI_VictorSPX a_arm;

  private DigitalInput armSwitchDown = new DigitalInput(MechConstants.aArmDownSwitch);
  private DigitalInput armSwitchUp = new DigitalInput(MechConstants.aArmUpSwitch);

  public armSubsystem() {
    a_arm = new WPI_VictorSPX(MechConstants.aArmPort);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void intakeDown(){
    a_arm.set(-SpeedConstants.aArmSpeed);
  }

  public void intakeUp(){
      a_arm.set(SpeedConstants.aArmSpeed);
  }

  public void intakeArmStop(){
    a_arm.set(0);
  }

  public boolean getstatusUp(){
    return armSwitchUp.get();
  }

  public boolean getstatusDown(){
    return armSwitchDown.get();
  }
  
}
