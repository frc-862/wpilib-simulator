// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package lightningrobotics.sim.subsystem;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.system.plant.DCMotor;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.VecBuilder;
import lightningrobotics.sim.SimulationConstants;
import lightningrobotics.sim.util.Battery;
import lightningrobotics.sim.util.FieldController;
import lightningrobotics.sim.util.PIDFController;

public class Drivetrain extends SubsystemBase {

  private DifferentialDrivetrainSim drivetrain;
  private DifferentialDriveOdometry odometry;

  private Encoder leftEncoder = new Encoder(0, 1);
  private Encoder rightEncoder = new Encoder(2, 3);
  private AnalogGyro gyro = new AnalogGyro(1);

  private EncoderSim leftEncoderSim = new EncoderSim(leftEncoder);
  private EncoderSim rightEncoderSim = new EncoderSim(rightEncoder);
  private AnalogGyroSim gyroSim = new AnalogGyroSim(gyro);

  private final PIDFController leftPidfController = new PIDFController(0.5 , 0, 0, 0);
  private final PIDFController rightPidfController = new PIDFController(0.5 , 0, 0, 0);

  public Drivetrain() {
    drivetrain = new DifferentialDrivetrainSim(
      DCMotor.getNEO(2),     // 2 NEO motors on each side of the drivetrain.
      7.29,                    // 7.29:1 gearing reduction.
      SimulationConstants.MOI,          
      SimulationConstants.ROBOT_MASS,     
      SimulationConstants.WHEEL_DIAMETER, 
      SimulationConstants.TRACK_WIDTH,       
      SimulationConstants.MEASUREMENT_NOISE);

      SmartDashboard.putNumber("left Kp", 0);
      SmartDashboard.putNumber("left Ki", 0);
      SmartDashboard.putNumber("left Kd", 0);

      odometry = 
      new DifferentialDriveOdometry(new Rotation2d(), new Pose2d(0,0,new Rotation2d()));
  }

  /**
   * Sets direct voltage input into each motor
   * @param leftVoltage
   * @param rightVoltage
   */
  public void setVoltage(double leftVoltage, double rightVoltage){
    drivetrain.setInputs(leftVoltage, rightVoltage);
    drivetrain.update(SimulationConstants.SIM_UPDATE_TIME);

   updateOdometry();
  } 

  public void setPercentInput(double leftPercent, double rightPercent){
    var leftVoltageInput = leftPercent * SimulationConstants.MAX_INPUT_VOLTAGE;
    var rightVoltageInput = rightPercent * SimulationConstants.MAX_INPUT_VOLTAGE;
    drivetrain.setInputs(leftVoltageInput, rightVoltageInput);
    drivetrain.update(SimulationConstants.SIM_UPDATE_TIME);

    updateOdometry();
  }
  /**
   * Sets the desired velocity and run through a PID loop
   * @param velocity desire velocity
   */ 
  
  public void setVelocity(double leftVelocity, double rightVelocity){
    var kp = SmartDashboard.getNumber("left Kp", 0);
    var ki = SmartDashboard.getNumber("left Ki", 0);
    var kd = SmartDashboard.getNumber("left Kd", 0);
    leftPidfController.setkP(kp);
    leftPidfController.setkI(ki);
    leftPidfController.setkD(kd);

    var leftMeasurement =leftEncoderSim.getRate();
    var rightMeasurement = rightEncoderSim.getRate();
    var leftInput = leftPidfController.calculate(leftMeasurement, leftVelocity);
    var rightInput = rightPidfController.calculate(rightMeasurement, rightVelocity);
    setPercentInput(leftInput, rightInput);

    SmartDashboard.putNumber("PID Measured Velocity", leftMeasurement);
    SmartDashboard.putNumber("PID Input Voltage", leftInput);
    SmartDashboard.putNumber("PID Set Velocity", leftVelocity);
  }

  private void updateOdometry(){
    gyroSim.setAngle(drivetrain.getPose().getRotation().getDegrees());
    odometry.update(
      Rotation2d.fromDegrees(gyroSim.getAngle()), 
      drivetrain.getPose().getX(),
      drivetrain.getPose().getY());

    leftEncoderSim.setDistance(drivetrain.getLeftPositionMeters());
    leftEncoderSim.setRate(drivetrain.getLeftVelocityMetersPerSecond());
    rightEncoderSim.setDistance(drivetrain.getRightPositionMeters());
    rightEncoderSim.setRate(drivetrain.getRightVelocityMetersPerSecond());

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
