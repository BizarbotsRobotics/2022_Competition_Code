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

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
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
  private Double ka = 0.11;
  private Double ks = .0005;
  private Double kv = 0.53;
  private BangBangController bangController = new BangBangController();
  private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(ks,kv,ka);
  // Encoder values
  private RelativeEncoder shooterRightEncoder;
  private RelativeEncoder shooterLeftEncoder;

  private Double setPercentage = 0.0;
  private  HashMap<Integer, TreeMap<Double, Integer>> shooterSpeeds;

  private double setSpeed = 0;
  private VisionSubsystem vision = new VisionSubsystem();

  ShuffleboardTab tab = Shuffleboard.getTab("Shooter Testing");

  private NetworkTableEntry speed = tab.add("speed", 0).getEntry();

  private SmartDashboard smartDashboard;

  private boolean speedLock = false;

  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;
  /** Creates a new ShooterSub. */
  public ShooterSubsystem() {

    speedLock = false;

    // Reset to current values
    shooterRight.restoreFactoryDefaults();
    shooterLeft.restoreFactoryDefaults();
    shooterRight.setSmartCurrentLimit(SHOOTER_CURRENT_LIMIT);
    shooterRight.setSmartCurrentLimit(SHOOTER_CURRENT_LIMIT);
    // shooterRight.setIdleMode(CANSparkMax.IdleMode.kCoast);
    // shooterLeft.setIdleMode(CANSparkMax.IdleMode.kCoast);


     // PID coefficients
    //  kP = 6e-5; 
    //  kI = 0;
    //  kD = 0; 
    //  kIz = 0; 
    //  kFF = 0.000015; 
    //  kMaxOutput = 1; 
    //  kMinOutput = -1;
    //  maxRPM = 5700;

    // Init PID controller
    shooterPidController = shooterRight.getPIDController();
    shooterPidController.setP(SHOOTER_kP);
    shooterPidController.setI(SHOOTER_kI);
    shooterPidController.setD(SHOOTER_kD);
    shooterPidController.setIZone(SHOOTER_kIz);
    shooterPidController.setFF(SHOOTER_kFF);
    shooterPidController.setOutputRange(SHOOTER_kMinOutput, SHOOTER_kMaxOutput);

    // display PID coefficients on SmartDashboard
    // SmartDashboard.putNumber("P Gain", kP);
    // SmartDashboard.putNumber("I Gain", kI);
    // SmartDashboard.putNumber("D Gain", kD);
    // SmartDashboard.putNumber("I Zone", kIz);
    // SmartDashboard.putNumber("Feed Forward", kFF);
    // SmartDashboard.putNumber("Max Output", kMaxOutput);
    // SmartDashboard.putNumber("Min Output", kMinOutput);
    // SmartDashboard.putNumber("SPEED", 0);

    // Init encoder
    shooterLeftEncoder = shooterLeft.getEncoder();
    shooterRightEncoder = shooterRight.getEncoder();

    shooterSpeeds =  new HashMap<Integer, TreeMap<Double, Integer>>();
    this.initValues();

    //this.shooterLeft.setInverted(true);
    this.shooterLeft.follow(shooterRight,true);

    this.closeAngle();

    this.setShooterSpeed(SHOOTER_DEFAULT_SPEED);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shooter Speed", shooterRightEncoder.getVelocity());

    // read PID coefficients from SmartDashboard
    // double p = SmartDashboard.getNumber("P Gain", 0);
    // double i = SmartDashboard.getNumber("I Gain", 0);
    // double d = SmartDashboard.getNumber("D Gain", 0);
    // double iz = SmartDashboard.getNumber("I Zone", 0);
    // double ff = SmartDashboard.getNumber("Feed Forward", 0);
    // double max = SmartDashboard.getNumber("Max Output", 0);
    // double min = SmartDashboard.getNumber("Min Output", 0);
    // double speed = SmartDashboard.getNumber("SPEED", 0);

    // // if PID coefficients on SmartDashboard have changed, write new values to controller
    // if((p != kP)) { shooterPidController.setP(p); kP = p; }
    // if((i != kI)) { shooterPidController.setI(i); kI = i; }
    // if((d != kD)) { shooterPidController.setD(d); kD = d; }
    // if((iz != kIz)) { shooterPidController.setIZone(iz); kIz = iz; }
    // if((ff != kFF)) { shooterPidController.setFF(ff); kFF = ff; }
    // if((max != kMaxOutput) || (min != kMinOutput)) { 
    //   shooterPidController.setOutputRange(min, max); 
    //   kMinOutput = min; kMaxOutput = max; 
    // }

    //double setPoint = speed;
    // shooterPidController.setReference(setPoint, CANSparkMax.ControlType.kVelocity);
    
    // SmartDashboard.putNumber("SetPoint", setPoint);
    // SmartDashboard.putNumber("ProcessVariable", shooterRightEncoder.getVelocity());
    //%this.handleShooterPower();
  }

  public void handleShooterPower() {
    if(this.vision.hasTarget() && !speedLock) {
      this.setShooterSpeed(this.calculateShooterSpeed(GOAL_TYPE.HIGH, POSITION.FAR));
    } else if(!speedLock) {
      this.setShooterSpeed(SHOOTER_DEFAULT_SPEED);
    }
  }

  public void setShooterLock(boolean lock) {
    this.speedLock = lock;
  }

  public void setShooterSpeed(int speed) {
    this.setSpeed = speed;
    shooterPidController.setReference(this.setSpeed, CANSparkMax.ControlType.kVelocity);
  }

  public void speedController() {
    //this.shooterRight.set(bangController.calculate(shooterRightEncoder.getVelocity(),this.setSpeed));
  }
  public boolean checkSpeed(){
    if(this.getShooterSpeed() >(this.setSpeed - SHOOTER_SPEED_RANGE) && this.getShooterSpeed() <(this.setSpeed + SHOOTER_SPEED_RANGE)){
      return true;
    } else {
      return false;
    }
  }
  public double getShooterSpeed(){
    return this.shooterRightEncoder.getVelocity();
  }

  public void shooterTesting() {
    shooterPidController.setReference(this.speed.getNumber(0).intValue(), CANSparkMax.ControlType.kVelocity);
  }

  public int calculateShooterSpeed(GOAL_TYPE goalType, POSITION position){
    if(position == POSITION.CLOSE) {
      if(goalType == GOAL_TYPE.HIGH){
        return this.shooterSpeeds.get(1).get(0.0);
      }
      else {
        return this.shooterSpeeds.get(3).get(0.0);
      }
    } else {
      double distance = this.vision.getDistanceToHubFeet();
      if(goalType == GOAL_TYPE.HIGH){
        Double key  = this.shooterSpeeds.get(0).ceilingKey(Math.round(distance*10)/10.0);
        return this.shooterSpeeds.get(0).get(key);
      } 
      else {
        Double key  = this.shooterSpeeds.get(0).higherKey(Math.round(distance*10)/10.0);
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
    this.angleChanger.set(Value.kReverse);
  }

  public void farAngle(){
    this.angleChanger.set(Value.kForward);
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
    shooterSpeeds.put(0, new TreeMap<Double, Integer>());
    shooterSpeeds.get(0).put(5d, 3500);
    shooterSpeeds.get(0).put(6d, 3400);
    shooterSpeeds.get(0).put(7d, 3700);
    shooterSpeeds.get(0).put(8d, 3750);
    shooterSpeeds.get(0).put(9d, 3900);
    shooterSpeeds.get(0).put(9.5, 4000);
    shooterSpeeds.get(0).put(10d, 4050);
    shooterSpeeds.get(0).put(10.5, 4150);
    shooterSpeeds.get(0).put(11d, 4250);
    shooterSpeeds.get(0).put(12d, 4250);
    shooterSpeeds.get(0).put(13d, 4400);
    shooterSpeeds.get(0).put(14d, 4500);
    shooterSpeeds.get(0).put(15d, 4750);
    shooterSpeeds.get(0).put(16d, 4850);
    shooterSpeeds.get(0).put(17d, 4950);
    shooterSpeeds.get(0).put(18d, 5050);
    shooterSpeeds.get(0).put(19d, 5150);
    shooterSpeeds.get(0).put(20d, 5250);
    shooterSpeeds.get(0).put(21d, 5350);
    shooterSpeeds.get(0).put(22d, 5450);

    //Initialize high hub, short distance
    shooterSpeeds.put(1, new TreeMap<Double, Integer>());
    shooterSpeeds.get(1).put(0d,5000);


    //Initialize low hub, far distance
    shooterSpeeds.put(2, new TreeMap<Double, Integer>());
    shooterSpeeds.get(2).put(7d, 2000);
    shooterSpeeds.get(2).put(8d, 2000);
    shooterSpeeds.get(2).put(9d, 2000);
    shooterSpeeds.get(2).put(10d, 2000);
    shooterSpeeds.get(2).put(11d, 2000);
    shooterSpeeds.get(2).put(12d, 2000);
    shooterSpeeds.get(2).put(13d, 2000);
    shooterSpeeds.get(2).put(14d, 2000);
    shooterSpeeds.get(2).put(15d, 2000);
    shooterSpeeds.get(2).put(16d, 2000);
    shooterSpeeds.get(2).put(17d, 2000);
    shooterSpeeds.get(2).put(18d, 2000);
    shooterSpeeds.get(2).put(19d, 2000);
    shooterSpeeds.get(2).put(20d, 2000);
    shooterSpeeds.get(2).put(21d, 2000);
    shooterSpeeds.get(2).put(22d, 2000);

    //Initialize low hub, short distance
    shooterSpeeds.put(3, new TreeMap<Double, Integer>());
    shooterSpeeds.get(3).put(0d,2200);
    shooterSpeeds.get(3).put(1d,3000);
    shooterSpeeds.get(3).put(2d,3000);
    shooterSpeeds.get(3).put(3d,3000);
    shooterSpeeds.get(3).put(4d,3000);
    shooterSpeeds.get(3).put(5d,3000);
    shooterSpeeds.get(3).put(6d,3000);
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