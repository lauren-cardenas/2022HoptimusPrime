// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class driveSubsystem extends SubsystemBase {
  /** Creates a new driveSubsystem. */
  private final WPI_TalonFX a_frontLeft = new WPI_TalonFX(DriveConstants.aLeftDrivePort1);
  private final WPI_TalonFX a_backLeft = new WPI_TalonFX(DriveConstants.aLeftDrivePort2);
  private final WPI_TalonFX a_frontRight = new WPI_TalonFX(DriveConstants.aRightDrivePort1);
  private final WPI_TalonFX a_backRight = new WPI_TalonFX(DriveConstants.aRightDrivePort2);

  private final MotorControllerGroup a_leftMotors = new MotorControllerGroup(a_frontLeft, a_backLeft);
  private final MotorControllerGroup a_rightMotors = new MotorControllerGroup(a_frontRight, a_backRight);

  //create drive boom
  private final DifferentialDrive a_drive = new DifferentialDrive(a_leftMotors,a_rightMotors);
  public driveSubsystem() {
    a_rightMotors.setInverted(true);
    

    a_frontLeft.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,0,0);
    a_backLeft.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,0,0);
    a_frontRight.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,0,0);
    a_backRight.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,0,0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void arcadeDrive(double fwd, double rot) {
    a_drive.arcadeDrive(fwd, rot);
    displayEncoderValues();
  }

  private double getLeftWheelSpeed(){
    return a_frontLeft.getSelectedSensorVelocity() * DriveConstants.Conversion;
  }

  private double getRightWheelSpeed(){
    return a_frontRight.getSelectedSensorVelocity() * DriveConstants.Conversion;
  }
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(getLeftWheelSpeed(), getRightWheelSpeed());
  }
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
  }
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    a_leftMotors.setVoltage(leftVolts);
    a_rightMotors.setVoltage(-rightVolts);
    a_drive.feed();
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    a_frontLeft.setSelectedSensorPosition(0);
    a_frontRight.setSelectedSensorPosition(0);
  }
  

  public double getAverageEncoderDistance() {
    return (getLeftWheelPosition() + getRightWheelPosition()) / 2;
  }

  public void displayEncoderValues(){
    SmartDashboard.putNumber("Right Data", getRightWheelPosition());
    SmartDashboard.putNumber("Left Data", getLeftWheelPosition());
  }

  private double getLeftWheelPosition(){
    return (a_frontLeft.getSelectedSensorPosition() * DriveConstants.aWheelDiameterMeters * Math.PI
     / DriveConstants.aEncoderCPR) / DriveConstants.aGearRatio;
  }
  
  private double getRightWheelPosition(){
    return (a_frontRight.getSelectedSensorPosition() * DriveConstants.aWheelDiameterMeters * Math.PI
     / DriveConstants.aEncoderCPR) / DriveConstants.aGearRatio;
  }

  public void setMaxOutput(double maxOutput) {
    a_drive.setMaxOutput(maxOutput);
  }
}
