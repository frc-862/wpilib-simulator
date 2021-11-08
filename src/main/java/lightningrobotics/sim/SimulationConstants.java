package lightningrobotics.sim;

import edu.wpi.first.wpilibj.system.plant.DCMotor;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpiutil.math.VecBuilder;
import edu.wpi.first.wpiutil.math.Vector;
import edu.wpi.first.wpiutil.math.numbers.N7;

public final class SimulationConstants {
    public static final double ROBOT_MASS = 60.0; //kg
    public static final double WHEEL_DIAMETER = Units.inchesToMeters(3); 
    public static final double TRACK_WIDTH = 0.7712; //meter
    public static final double MOI = 7.5; //Moment of inertia, J * kg * m^2

    public static final double MAX_INPUT_VOLTAGE = 12;
     // The standard deviations for measurement noise:
      // x and y:          meters
      // heading:          rad
      // l and r velocity: m/s
      // l and r position: meters
    public static final Vector<N7> MEASUREMENT_NOISE = 
    VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005); 

    public static double SIM_UPDATE_TIME = 0.02d;
}