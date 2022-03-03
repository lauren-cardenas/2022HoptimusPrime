// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
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

  private final Gyro a_gyro = new AHRS();


  private DifferentialDriveOdometry a_odometry;

  //create drive boom
  private final DifferentialDrive a_drive = new DifferentialDrive(a_leftMotors,a_rightMotors);
  public driveSubsystem() {
    a_rightMotors.setInverted(true);

    

    a_frontLeft.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,0,0);
    a_backLeft.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,0,0);
    a_frontRight.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,0,0);
    a_backRight.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,0,0);

    a_frontLeft.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, DriveConstants.aAmpLimit, DriveConstants.aTriggerThreshold-10, DriveConstants.aTriggerTime/2));
    a_frontRight.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, DriveConstants.aAmpLimit, DriveConstants.aTriggerThreshold-10, DriveConstants.aTriggerTime/2));
    a_backLeft.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, DriveConstants.aAmpLimit, DriveConstants.aTriggerThreshold-10, DriveConstants.aTriggerTime/2));
    a_backRight.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, DriveConstants.aAmpLimit, DriveConstants.aTriggerThreshold-10, DriveConstants.aTriggerTime/2));

    // a_frontLeft.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, DriveConstants.aAmpLimit, DriveConstants.aTriggerThreshold, DriveConstants.aTriggerTime));
    // a_frontRight.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, DriveConstants.aAmpLimit, DriveConstants.aTriggerThreshold, DriveConstants.aTriggerTime));
    // a_backLeft.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, DriveConstants.aAmpLimit, DriveConstants.aTriggerThreshold, DriveConstants.aTriggerTime));
    // a_backRight.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, DriveConstants.aAmpLimit, DriveConstants.aTriggerThreshold, DriveConstants.aTriggerTime));

    a_gyro.reset();
    a_odometry = new DifferentialDriveOdometry(a_gyro.getRotation2d());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    a_odometry.update(
      a_gyro.getRotation2d(), getLeftWheelPosition(), getRightWheelPosition());
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
    zeroHeading();
    a_odometry.resetPosition(pose, a_gyro.getRotation2d());
  }
  public Pose2d getPose() {
    return a_odometry.getPoseMeters();
  }
  public void zeroHeading() {
    a_gyro.reset();
  }
  public double getHeading() {
    return a_gyro.getRotation2d().getDegrees();
  }
  public double getTurnRate() {
    return -a_gyro.getRate();
  }
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    a_leftMotors.setVoltage(leftVolts);
    a_rightMotors.setVoltage(rightVolts);
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
    return (-a_frontRight.getSelectedSensorPosition() * DriveConstants.aWheelDiameterMeters * Math.PI
     / DriveConstants.aEncoderCPR) / DriveConstants.aGearRatio;
  }

  public void setMaxOutput(double maxOutput) {
    a_drive.setMaxOutput(maxOutput);
  }

}
