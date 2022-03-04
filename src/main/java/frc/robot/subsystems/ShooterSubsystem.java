// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.*;

import java.util.Dictionary;
import java.util.TreeMap;
import java.util.Map;
import java.util.HashMap;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
  // Motors
  
  private final CANSparkMax shooterLeft = new CANSparkMax(SHOOTER_ID_LEFT,MotorType.kBrushless);
  private final CANSparkMax shooterRight = new CANSparkMax(SHOOTER_ID_RIGHT,MotorType.kBrushless);

  // angle changer
  private final DoubleSolenoid angleChanger = new DoubleSolenoid(PCM_TYPE,SHOOTER_SOLENOID_PORTS[0],SHOOTER_SOLENOID_PORTS[1]);

  // PID stuff
  private SparkMaxPIDController shooterPidController;

  // Encoder values
  private RelativeEncoder shooterRightEncoder;
  private RelativeEncoder shooterLeftEncoder;

  private  HashMap<Integer, TreeMap<Integer, Integer>> shooterSpeeds;


  private VisionSubsystem vision = new VisionSubsystem();

  ShuffleboardTab tab = Shuffleboard.getTab("Shooter Testing");

  private NetworkTableEntry speed = tab.add("speed", 0).getEntry();

  /** Creates a new ShooterSub. */
  public ShooterSubsystem() {

    

    // Reset to current values
    shooterRight.restoreFactoryDefaults();
    shooterLeft.restoreFactoryDefaults();
    shooterRight.setSmartCurrentLimit(SHOOTER_CURRENT_LIMIT);
    shooterRight.setSmartCurrentLimit(SHOOTER_CURRENT_LIMIT);

    // Init PID controller
    shooterPidController = shooterRight.getPIDController();
    shooterPidController.setP(SHOOTER_kP);
    shooterPidController.setI(SHOOTER_kI);
    shooterPidController.setD(SHOOTER_kD);
    shooterPidController.setIZone(SHOOTER_kIz);
    shooterPidController.setFF(SHOOTER_kFF);
    shooterPidController.setOutputRange(SHOOTER_kMinOutput, SHOOTER_kMaxOutput);

    // Init encoder
    shooterLeftEncoder = shooterLeft.getEncoder();
    shooterRightEncoder = shooterRight.getEncoder();

    shooterSpeeds =  new HashMap<Integer, TreeMap<Integer, Integer>>();
    this.initValues();

    //this.shooterLeft.setInverted(true);
    this.shooterLeft.follow(shooterRight,true);

    this.closeAngle();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shooter Speed", shooterRightEncoder.getVelocity());
    // This method will be called once per scheduler run
  }

  public void setShooterSpeed(int speed) {
    shooterPidController.setReference(speed, CANSparkMax.ControlType.kVelocity);
  }

  public void shooterTesting() {
    shooterPidController.setReference(this.speed.getNumber(0).intValue(), CANSparkMax.ControlType.kVelocity);
  }

  public int calculateShooterSpeed(GOAL_TYPE goalType, POSITION position){
    if(position == POSITION.CLOSE) {
      if(goalType == GOAL_TYPE.HIGH){
        return this.shooterSpeeds.get(1).get(0);
      }
      else {
        return this.shooterSpeeds.get(3).get(0);
      }
    } else {
      double distance = this.vision.getDistanceToHub();
      if(goalType == GOAL_TYPE.HIGH){
        int key  = this.shooterSpeeds.get(0).ceilingKey((int)distance);
        return this.shooterSpeeds.get(0).get(key);
      } 
      else {
        int key  = this.shooterSpeeds.get(0).ceilingKey((int)distance);
        return this.shooterSpeeds.get(2).get(key);
      }
    }
  }



  public void setShooterPercentage(Double percentage) {
    this.shooterRight.set(percentage);
  }

  public void runShooter(){
    //this.shooterLeft.set(1);
    this.shooterRight.set(.9);
  }

  public void shooterStop(){
    this.shooterRight.set(0);
    //this.shooterLeft.set(0);
  }

  public void closeAngle(){
    this.angleChanger.set(Value.kForward);
  }

  public void farAngle(){
    this.angleChanger.set(Value.kReverse);
  }

  public void toggleAngle(){
    this.angleChanger.toggle();
  }

  public DoubleSolenoid.Value getAngle() {
    return this.angleChanger.get();
  }

  private void initValues() {
    /*
    *High hub, far distance = 0
    *High hub, short distance = 1
    *Low hub, far distance = 2
    *Low hub, short distance = 3
    */

    //Initialize high hub, far distance
    shooterSpeeds.put(0, new TreeMap<Integer, Integer>());
    shooterSpeeds.get(0).put(7, 2000);
    shooterSpeeds.get(0).put(8, 2000);
    shooterSpeeds.get(0).put(9, 2000);
    shooterSpeeds.get(0).put(10, 2000);
    shooterSpeeds.get(0).put(11, 2000);
    shooterSpeeds.get(0).put(12, 2000);
    shooterSpeeds.get(0).put(13, 2000);
    shooterSpeeds.get(0).put(14, 2000);
    shooterSpeeds.get(0).put(15, 2000);
    shooterSpeeds.get(0).put(16, 2000);
    shooterSpeeds.get(0).put(17, 2000);
    shooterSpeeds.get(0).put(18, 2000);
    shooterSpeeds.get(0).put(19, 2000);
    shooterSpeeds.get(0).put(20, 2000);
    shooterSpeeds.get(0).put(21, 2000);
    shooterSpeeds.get(0).put(22, 2000);

    //Initialize high hub, short distance
    shooterSpeeds.put(1, new TreeMap<Integer, Integer>());
    shooterSpeeds.get(1).put(0,1000);
    shooterSpeeds.get(1).put(1,1000);
    shooterSpeeds.get(1).put(2,1000);
    shooterSpeeds.get(1).put(3,1000);
    shooterSpeeds.get(1).put(4,1000);
    shooterSpeeds.get(1).put(5,1000);
    shooterSpeeds.get(1).put(6,1000);

    //Initialize low hub, far distance
    shooterSpeeds.put(2, new TreeMap<Integer, Integer>());
    shooterSpeeds.get(2).put(7, 2000);
    shooterSpeeds.get(2).put(8, 2000);
    shooterSpeeds.get(2).put(9, 2000);
    shooterSpeeds.get(2).put(10, 2000);
    shooterSpeeds.get(2).put(11, 2000);
    shooterSpeeds.get(2).put(12, 2000);
    shooterSpeeds.get(2).put(13, 2000);
    shooterSpeeds.get(2).put(14, 2000);
    shooterSpeeds.get(2).put(15, 2000);
    shooterSpeeds.get(2).put(16, 2000);
    shooterSpeeds.get(2).put(17, 2000);
    shooterSpeeds.get(2).put(18, 2000);
    shooterSpeeds.get(2).put(19, 2000);
    shooterSpeeds.get(2).put(20, 2000);
    shooterSpeeds.get(2).put(21, 2000);
    shooterSpeeds.get(2).put(22, 2000);

    //Initialize low hub, short distance
    shooterSpeeds.put(3, new TreeMap<Integer, Integer>());
    shooterSpeeds.get(3).put(0,1000);
    shooterSpeeds.get(3).put(1,1000);
    shooterSpeeds.get(3).put(2,1000);
    shooterSpeeds.get(3).put(3,1000);
    shooterSpeeds.get(3).put(4,1000);
    shooterSpeeds.get(3).put(5,1000);
    shooterSpeeds.get(3).put(6,1000);
  }


  public enum POSITION {
    CLOSE,
    FAR
  }

  public enum GOAL_TYPE {
    HIGH,
    LOW
  }
}