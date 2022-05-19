/**
* This is a very simple robot program that can be used to send telemetry to
* the data_logger script to characterize your drivetrain. If you wish to use
* your actual robot code, you only need to implement the simple logic in the
* autonomousPeriodic function and change the NetworkTables update rate
*/

package lightningrobotics.sim;

import java.util.stream.IntStream;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.JoystickSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.XboxControllerSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import lightningrobotics.sim.command.DriveRight;
import lightningrobotics.sim.command.MoveElevator;
import lightningrobotics.sim.subsystem.Drivetrain;
import lightningrobotics.sim.subsystem.Elevator;
import lightningrobotics.sim.util.FieldController;

public class Robot extends TimedRobot {
  
  private static final Drivetrain drivetrain = new Drivetrain();
  private static final Elevator elevator = new Elevator(DCMotor.getVex775Pro(4), 10, 4, Units.inchesToMeters(2.0), Units.inchesToMeters(2), Units.inchesToMeters(50));


  @Override
  public void testInit() {
  
  }
  @Override
  public void testPeriodic() {
   
  }
  @Override
  public void simulationInit() {
  
  }

  @Override
  public void simulationPeriodic() {
      // Important, do not remove
      CommandScheduler.getInstance().run();
  }

  @Override
  public void teleopInit() {
    // Add field to dashboard
    FieldController.Initialize();

    SmartDashboard.putData("To the Right", new DriveRight(drivetrain));
    SmartDashboard.putData("LIFT!!!", new MoveElevator(elevator));
  }

  @Override
  public void teleopPeriodic() {

    
  }

}
