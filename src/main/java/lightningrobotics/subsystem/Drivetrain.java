// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package lightningrobotics.subsystem;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.system.plant.DCMotor;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.VecBuilder;
import lightningrobotics.SimulationConstants;
import lightningrobotics.util.Battery;
import lightningrobotics.util.FieldController;

public class Drivetrain extends SubsystemBase {

  private DifferentialDrivetrainSim drivetrain;
  private DifferentialDriveOdometry odometry;

  public Drivetrain() {
    drivetrain = new DifferentialDrivetrainSim(
      DCMotor.getNEO(2),     // 2 NEO motors on each side of the drivetrain.
      7.29,                    // 7.29:1 gearing reduction.
      SimulationConstants.MOI,          
      SimulationConstants.ROBOT_MASS,     
      SimulationConstants.WHEEL_DIAMETER, 
      SimulationConstants.TRACK_WIDTH,       
      SimulationConstants.MEASUREMENT_NOISE);

      odometry = 
      new DifferentialDriveOdometry(new Rotation2d(), new Pose2d(0,0,new Rotation2d()));
  }

  public void SetVoltage(double leftVoltage, double rightVoltage){
    drivetrain.setInputs(leftVoltage, rightVoltage);
    drivetrain.update(SimulationConstants.SIM_UPDATE_TIME);

    odometry.update(
      new Rotation2d(), 
      drivetrain.getPose().getX(),
      drivetrain.getPose().getY());

    FieldController.SetRobotPosition(odometry.getPoseMeters());
    Battery.UseBattery(drivetrain);
  } 


  public Pose2d GetPosition(){
    return drivetrain.getPose();
  } 

  @Override
  public void periodic() {

  }
}
