// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package lightningrobotics.sim.command;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import lightningrobotics.sim.subsystem.Elevator;

public class OscilateElevator extends CommandBase {
  /** Creates a new MoveElevator. */
  private Elevator elevator;
  private static boolean elevatorMovingDown = false;

  public OscilateElevator(Elevator elevator) {
    this.elevator = elevator;
    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!elevatorMovingDown) {
      elevator.setVoltage(20d);
      if (elevator.getHeight() > Units.inchesToMeters(47)) { elevatorMovingDown = true; }
    } else {
      elevator.stop();
      if (elevator.getHeight() < Units.inchesToMeters(3)) { elevatorMovingDown = false; }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevator.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
