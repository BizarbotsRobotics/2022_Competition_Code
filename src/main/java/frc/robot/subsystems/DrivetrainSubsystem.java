// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.swervedrivespecialties.swervelib.Mk4ModuleConfiguration;
import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.*;

public class DrivetrainSubsystem extends SubsystemBase {
        private static DrivetrainSubsystem instance = null;
        public static final double AUTO_DRIVE_SCALE = 1;
        public static final double MAX_VOLTAGE = 10.0;
        public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0 *
          SdsModuleConfigurations.MK4_L1.getDriveReduction() *
          SdsModuleConfigurations.MK4_L1.getWheelDiameter() * Math.PI;

  public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
          Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0);

  private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
          // Front left
          new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
          // Front right
          new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0),
          // Back left
          new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
          // Back right
          new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0)
  );


  private final Pigeon2 m_pigeon = new Pigeon2(DRIVETRAIN_PIGEON_ID);


  private NetworkTableEntry odometryEntry;

  private boolean lockInPlace_ = false;

  private SwerveModule m_frontLeftModule;
  private SwerveModule m_frontRightModule;
  private SwerveModule m_backLeftModule;
  private SwerveModule m_backRightModule;

  private CANCoder m_frontLeftCanCoder;
  private CANCoder m_frontRightCanCoder;
  private CANCoder m_backLeftCanCoder;
  private CANCoder m_backRightCanCoder;

  //private final Object kinematicsLock = new Object();
  //private final SwerveOdometry swerveOdometry = new SwerveOdometry(swerveKinematics, RigidTransform2.ZERO);
  private Pose2d pose =  new Pose2d(0.0,0.0, new Rotation2d());
//   private final InterpolatingTreeMap<InterpolatingDouble, RigidTransform2> latencyCompensationMap = new InterpolatingTreeMap<>();
//   private Vector2 velocity = Vector2.ZERO;
//   private double angularVelocity = 0.0;

SwerveDriveOdometry m_odometry;
  private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

  public DrivetrainSubsystem() {
        m_odometry = new SwerveDriveOdometry(m_kinematics, getGyroscopeRotation());
          this.initEncoders();
    this.initMotors();

    
    
  }

  private void initEncoders() {
        m_frontLeftCanCoder = new CANCoder(FRONT_LEFT_MODULE_STEER_ENCODER);
        m_frontRightCanCoder = new CANCoder(FRONT_RIGHT_MODULE_STEER_ENCODER);
        m_backLeftCanCoder = new CANCoder(BACK_LEFT_MODULE_STEER_ENCODER);
        m_backRightCanCoder = new CANCoder(BACK_RIGHT_MODULE_STEER_ENCODER);

        m_frontLeftCanCoder.configFactoryDefault();
        m_frontRightCanCoder.configFactoryDefault();
        m_backLeftCanCoder.configFactoryDefault();
        m_backRightCanCoder.configFactoryDefault();

        m_frontLeftCanCoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
        m_frontRightCanCoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
        m_backLeftCanCoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
        m_backRightCanCoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);

        m_frontLeftCanCoder.configSensorDirection(true);
        m_frontRightCanCoder.configSensorDirection(true);
        m_backLeftCanCoder.configSensorDirection(true);
        m_backRightCanCoder.configSensorDirection(true);

        m_frontLeftCanCoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
        m_frontRightCanCoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
        m_backLeftCanCoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
        m_backRightCanCoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);

        m_pigeon.clearStickyFaults();
  }

  private void initMotors() {
        ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");
        Mk4ModuleConfiguration mk4Config = new Mk4ModuleConfiguration();
        mk4Config.setDriveCurrentLimit(40);
        mk4Config.setNominalVoltage(10);
        mk4Config.setSteerCurrentLimit(20);
        this.m_frontLeftModule = Mk4SwerveModuleHelper.createNeo(
                // This parameter is optional, but will allow you to see the current state of the module on the dashboard.
                tab.getLayout("Front Left Module", BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(0, 0)
            ,mk4Config,
                // This can either be STANDARD or FAST depending on your gear configuration
                Mk4SwerveModuleHelper.GearRatio.L1,
                // This is the ID of the drive motor
                FRONT_LEFT_MODULE_DRIVE_MOTOR,
                // This is the ID of the steer motor
                FRONT_LEFT_MODULE_STEER_MOTOR,
                // This is the ID of the steer encoder
                FRONT_LEFT_MODULE_STEER_ENCODER,
                // This is how much the steer encoder is offset from true zero (In our case, zero is facing straight forward)
                FRONT_LEFT_MODULE_STEER_OFFSET
        );
    
        // We will do the same for the other modules
        m_frontRightModule = Mk4SwerveModuleHelper.createNeo(
                tab.getLayout("Front Right Module", BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(2, 0),
                        mk4Config,
                Mk4SwerveModuleHelper.GearRatio.L1,
                FRONT_RIGHT_MODULE_DRIVE_MOTOR,
                FRONT_RIGHT_MODULE_STEER_MOTOR,
                FRONT_RIGHT_MODULE_STEER_ENCODER,
                FRONT_RIGHT_MODULE_STEER_OFFSET
        );
    
        m_backLeftModule = Mk4SwerveModuleHelper.createNeo(
                tab.getLayout("Back Left Module", BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(4, 0),
                        mk4Config,
                Mk4SwerveModuleHelper.GearRatio.L1,
                BACK_LEFT_MODULE_DRIVE_MOTOR,
                BACK_LEFT_MODULE_STEER_MOTOR,
                BACK_LEFT_MODULE_STEER_ENCODER,
                BACK_LEFT_MODULE_STEER_OFFSET
        );
    
        m_backRightModule = Mk4SwerveModuleHelper.createNeo(
                tab.getLayout("Back Right Module", BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(6, 0),
                        mk4Config,
                Mk4SwerveModuleHelper.GearRatio.L1,
                BACK_RIGHT_MODULE_DRIVE_MOTOR,
                BACK_RIGHT_MODULE_STEER_MOTOR,
                BACK_RIGHT_MODULE_STEER_ENCODER,
                BACK_RIGHT_MODULE_STEER_OFFSET
        );
        m_frontLeftModule.set(0, 0);
        m_frontRightModule.set(0, 0);
        m_backLeftModule.set(0, 0);
        m_backRightModule.set(0, 0);
  }
  /**
   * Sets the gyroscope angle to zero. This can be used to set the direction the robot is currently facing to the
   * 'forwards' direction.
   */
  public void zeroGyroscope() {
        m_pigeon.setYaw(0.0);
  }

  public Rotation2d getGyroscopeRotation() {
    return Rotation2d.fromDegrees(m_pigeon.getYaw());
  }

  public double getGyroPitch() {
        return m_pigeon.getPitch();
        }

  public void drive(ChassisSpeeds chassisSpeeds) {
    m_chassisSpeeds = chassisSpeeds;
  }

  public void driveAuto(ChassisSpeeds chassisSpeeds) {
        m_chassisSpeeds.vxMetersPerSecond = chassisSpeeds.vxMetersPerSecond * AUTO_DRIVE_SCALE;
        m_chassisSpeeds.vyMetersPerSecond = chassisSpeeds.vyMetersPerSecond * AUTO_DRIVE_SCALE;
        m_chassisSpeeds.omegaRadiansPerSecond = chassisSpeeds.omegaRadiansPerSecond;
        
    }

    public void actuateModulesAuto(SwerveModuleState[] states){
        driveAuto(m_kinematics.toChassisSpeeds(states));
    }

  public void rotate(double angle) {
        ChassisSpeeds.fromFieldRelativeSpeeds(0.0,0.0,0.0,this.getGyroscopeRotation());
        //m_frontLeftModule.s
  }

  public double getAverageAbsoluteValueVelocity() {
        double averageVelocity = 0;
        averageVelocity += Math.abs(m_frontLeftModule.getDriveVelocity());
        averageVelocity += Math.abs(m_frontRightModule.getDriveVelocity());
        averageVelocity += Math.abs(m_backLeftModule.getDriveVelocity());
        averageVelocity += Math.abs(m_backRightModule.getDriveVelocity());
        return averageVelocity / 4;
}

  @Override
  public void periodic() {
        //odometryEntry.setString(getCurrentPose().toString());
        // Update the pose
        SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(m_chassisSpeeds);
        //this.pose = m_odometry.update(gyroAngle, states[0], states[1],
        //states[2], states[3]);
        
        
        SmartDashboard.putNumber("Gyro", -m_pigeon.getYaw());

        SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);

        
        m_frontLeftModule.set(states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[0].angle.getRadians());
        m_frontRightModule.set(states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[1].angle.getRadians());
        m_backLeftModule.set(states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[2].angle.getRadians());
        m_backRightModule.set(states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[3].angle.getRadians());
  
        states[0].speedMetersPerSecond = Math.abs(m_frontLeftModule.getDriveVelocity());
       states[1].speedMetersPerSecond = Math.abs(m_frontRightModule.getDriveVelocity());
       states[2].speedMetersPerSecond = Math.abs(m_backLeftModule.getDriveVelocity());
       states[3].speedMetersPerSecond = Math.abs(m_backRightModule.getDriveVelocity());
       m_odometry.update(getGyroscopeRotation(), states);
}
public SwerveDriveKinematics getKinematics(){
        return m_kinematics;
    }

    /**
     * Returns the current position of the robot from the Odometry
     */
    public Pose2d getCurrentPose(){
        return m_odometry.getPoseMeters();
    }

    /**
     * Resets the Odometry to the specified Pose
     * ONLY USE IF YOU KNOW WHAT YOU ARE DOING
     * @param pose
     *  The Pose to reset the odometry to
     */
    public void resetOdometry(Pose2d pose){
        m_odometry.resetPosition(pose, pose.getRotation());
    }

    public void setLockInPlace(boolean lock) {
        lockInPlace_ = lock ? true : false;
    }

    public static DrivetrainSubsystem getInstance() {
        if (instance == null) {
            instance = new DrivetrainSubsystem();
        }

        return instance;
    }

}
