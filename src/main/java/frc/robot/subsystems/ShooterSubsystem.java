// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.*;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
  // Motors
  
  private final CANSparkMax shooterLeft = new CANSparkMax(SHOOTER_ID_LEFT,MotorType.kBrushless);
  private final CANSparkMax shooterRight = new CANSparkMax(SHOOTER_ID_RIGHT,MotorType.kBrushless);

  // angle changer
  //private final DoubleSolenoid angleChanger = new DoubleSolenoid(PCM_TYPE,SHOOTER_SOLENOID_PORTS[0],SHOOTER_SOLENOID_PORTS[1]);

  // PID stuff
  private SparkMaxPIDController shooterPidController;

  // Encoder values
  private RelativeEncoder shooterRightEncoder;
  private RelativeEncoder shooterLeftEncoder;

  /** Creates a new ShooterSub. */
  public ShooterSubsystem() {

    // Reset to current values
    shooterRight.restoreFactoryDefaults();
    shooterLeft.restoreFactoryDefaults();
    shooterRight.setSmartCurrentLimit(SHOOTER_CURRENT_LIMIT);
    shooterRight.setSmartCurrentLimit(SHOOTER_CURRENT_LIMIT);

    // Init PID controller
    shooterPidController = shooterRight.getPIDController();

    // Init encoder
    shooterLeftEncoder = shooterLeft.getEncoder();
    shooterRightEncoder = shooterRight.getEncoder();



    //this.shooterLeft.setInverted(true);
    this.shooterLeft.follow(shooterRight,true);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shooter Speed", shooterRightEncoder.getVelocity());
    // This method will be called once per scheduler run
  }

  public void setShooterSpeed(int speed) {
    shooterPidController.setReference(speed, CANSparkMax.ControlType.kVelocity);
  }

  public void runShooter(){
    //this.shooterLeft.set(1);
    this.shooterRight.set(.9);
  }

  public void shooterStop(){
    this.shooterRight.set(0);
    //this.shooterLeft.set(0);
  }

  //public void 85angle()
}