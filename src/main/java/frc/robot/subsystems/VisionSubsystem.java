// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Limelight;
import static frc.robot.Constants.*;


public class VisionSubsystem extends SubsystemBase {
        private static final double TARGET_HEIGHT = 104
        ;
        private static final double LIMELIGHT_HEIGHT = 19;

        private static final double LIMELIGHT_MOUNTING_ANGLE = 30;

        private static final double TARGET_ALLOWABLE_ERROR = 2.5;

        private static final Limelight LIMELIGHT = new Limelight("shooter");

        private double firstError;

        private double limelightOffset;
        private double lastDistance;
        private double distanceToHub = 0;

        private double horizontalOffset = 0;

        private boolean hasTarget;
        private boolean isAligned;

        public VisionSubsystem() {
                lastDistance = 0.0;
                firstError = this.getHorizontalOffset();
        }

        @Override
        public void periodic() {
                this.hasTarget = LIMELIGHT.hasTarget();
                if (this.hasTarget){
                        double verticalOffset = LIMELIGHT.getVerticalOffset();
                        this.horizontalOffset = LIMELIGHT.getHorizontalOffset();
                        double theta = (LIMELIGHT_MOUNTING_ANGLE + verticalOffset)*(3.14159/180);
                        this.distanceToHub = (TARGET_HEIGHT - LIMELIGHT_HEIGHT) / Math.tan(theta);
                        //System.out.println(distanceToHub);
                } 

                if (Math.abs(this.getHorizontalOffset()) <= TARGET_ALLOWABLE_ERROR) {
                        this.isAligned = true;
                    } else {
                        this.isAligned = false;
                    }
                    
                    SmartDashboard.putBoolean("isAligned", this.isAligned);
                    SmartDashboard.putNumber("Distance", this.distanceToHub);

        }

        public boolean hasTarget() {
                return hasTarget;
        }

        public double getDistanceToHub(){
                return this.distanceToHub;
        }

        public double getDistanceToHubFeet(){
                return this.distanceToHub/12;
        }


        public double getHorizontalOffset() {
                return this.horizontalOffset;
        }

        public void setLedMode(Limelight.LedMode mode) {
                LIMELIGHT.setLedMode(mode);
        }
        
        public void setSnapshotEnabled(boolean isEnabled) {
                LIMELIGHT.setSnapshotsEnabled(isEnabled);
        }

        public void setCamMode(Limelight.CamMode mode) {
                LIMELIGHT.setCamMode(mode);
        }

        
        public boolean isAligned() {
            return this.isAligned;
        }

        public void setLimelightOffset() {

                double offset = -this.getHorizontalOffset();
                double angularSpeed = (offset * GOAL_ALIGN_KP + Math.abs(offset - firstError) * GOAL_ALIGN_KD) * 2; //TODO make this a constant pls
        
                if (this.isAligned() == true) {
                    limelightOffset = 0;
                } else {
                    if (this.getHorizontalOffset() <= 0)
                        limelightOffset = -angularSpeed;
        
                    if (this.getHorizontalOffset() >= 0) 
                        limelightOffset = -angularSpeed;
                }
                System.out.println(offset);
                SmartDashboard.putNumber("Angulat speed", limelightOffset);
            }
        
            public void setLimelightOffset(double value) {
                limelightOffset = value;
            }

            public double getLimelightOffset() {
                return this.limelightOffset;
            }
            
        public static VisionSubsystem instance_ = null;
            public static VisionSubsystem getInstance() {
                if (instance_ == null) {
                    instance_ = new VisionSubsystem();
                }
        
                return instance_;
            }
        
}
