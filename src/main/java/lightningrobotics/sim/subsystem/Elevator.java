// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package lightningrobotics.sim.subsystem;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lightningrobotics.sim.SimulationConstants;

public class Elevator extends SubsystemBase {

  private Encoder m_encoder = new Encoder(4, 5);
  private PWMSparkMax m_motor = new PWMSparkMax(SimulationConstants.ELEVATOR_PORT);

  private ElevatorSim m_elevatorSim;
  private EncoderSim m_encoderSim;

  private final Mechanism2d m_mech2d = new Mechanism2d(20, 50);
  private final MechanismRoot2d m_mech2dRoot = m_mech2d.getRoot("Elevator Root", 10, 0);
  private MechanismLigament2d m_elevatorMech2d;
  /** Creates a new Elevator. */
  public Elevator(DCMotor elevatorMotor, double gearing, double carriageMassKg, double drumRadiusMeters, double minHeightMeters, double maxHeightMeters) {
    m_elevatorSim =
        new ElevatorSim(
            elevatorMotor,
            gearing,
            carriageMassKg,
            drumRadiusMeters,
            minHeightMeters,
            maxHeightMeters,
            VecBuilder.fill(0.01));
    m_encoderSim = new EncoderSim(m_encoder);
    m_elevatorMech2d =
      m_mech2dRoot.append(
          new MechanismLigament2d(
              "Elevator", Units.metersToInches(m_elevatorSim.getPositionMeters()), 90));
    m_encoder.setDistancePerPulse(2.0 * Math.PI * drumRadiusMeters / 4096);

    // Publish Mechanism2d to SmartDashboard
    SmartDashboard.putData("Elevator Sim", m_mech2d);
  }


  @Override
  public void periodic() {
    // In this method, we update our simulation of what our elevator is doing
    // First, we set our "inputs" (voltages)
    m_elevatorSim.setInput(m_motor.get() * RobotController.getBatteryVoltage());

    // Next, we update it. The standard loop time is 20ms.
    m_elevatorSim.update(0.020);

    // Finally, we set our simulated encoder's readings and simulated battery voltage
    m_encoderSim.setDistance(m_elevatorSim.getPositionMeters());
    // SimBattery estimates loaded battery voltages
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(m_elevatorSim.getCurrentDrawAmps()));

    // Update elevator visualization with simulated position
     m_elevatorMech2d.setLength(Units.metersToInches(m_elevatorSim.getPositionMeters()));
  }
  
  public void setVoltage(double voltage) {
    m_motor.setVoltage(voltage);
  }

  public void stop() {
    setVoltage(0);
  }

}
