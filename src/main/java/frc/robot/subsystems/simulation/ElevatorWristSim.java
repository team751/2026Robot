package frc.robot.subsystems.simulation;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ElevatorWristSim {
  private static ElevatorWristSim instance;

  private static final double elevatorXPosition = 0;
  private static final double elevatorYPosition = 0;
  private static final double elevatorWidth = 0;
  private static final double elevatorHeight = 0;
  private static final double elevatorMaxHeight = 0;

  private static final double elevatorS1MinHeight = 0.0;
  private static final double elevatorS2MinHeight = 0.0;
  private static final double elevatorS3MinHeight = 0.0;

  private static final double elevatorS1MaxHeight = 0.0;
  private static final double elevatorS2MaxHeight = 0.0;
  private static final double elevatorS3MaxHeight = 0.0;

  private final Mechanism2d elevatorWrist = new Mechanism2d(elevatorWidth, elevatorHeight);
  private final MechanismRoot2d elevatorRoot;
  private final MechanismLigament2d elevatorS1 =
      new MechanismLigament2d("ElevS1", elevatorS1MinHeight, elevatorS1MaxHeight);
  private final MechanismLigament2d elevatorS2 =
      new MechanismLigament2d("ElevS2", elevatorS2MinHeight, elevatorS2MaxHeight);
  private final MechanismLigament2d elevatorS3 =
      new MechanismLigament2d("ElevS3", elevatorS3MinHeight, elevatorS3MaxHeight);

  private static final double wristMaxAngle = 0.0;
  private static final double wristMinAngle = 0.0;
  private final MechanismLigament2d wristRoot =
      new MechanismLigament2d("Wrist", wristMinAngle, wristMaxAngle);

  private final NetworkTable elevatorTable =
      NetworkTableInstance.getDefault().getTable("ElevatorWrist");
  private final Timer stateStart = new Timer();
  private double stateChangeTime = 0.0;

  //    private final ElevatorSim elevator;
  //    private final SingleJointedArmSim wrist;

  private String currentState = "";

  public static ElevatorWristSim getInstance() {
    if (instance == null) instance = new ElevatorWristSim();
    return instance;
  }

  private ElevatorWristSim() {
    elevatorRoot = elevatorWrist.getRoot("Elevator", elevatorXPosition, elevatorYPosition);
    elevatorRoot.append(elevatorS1);
    elevatorS1.append(elevatorS2);
    elevatorS2.append(elevatorS3);
    elevatorS3.append(wristRoot);

    //        elevator = new ElevatorSim(
    //            DCMotor.getKrakenX60Foc(2),
    //            0.0,
    //            0.0,
    //            0.0,
    //            0.0,
    //            10.0,
    //            true,
    //            0.0
    //        );
    //        wrist = new SingleJointedArmSim(
    //            DCMotor.getKrakenX60Foc(1),
    //            20.0,
    //            0.0,
    //            0.0,
    //            0.0,
    //            0.0,
    //            true,
    //            0.0
    //        );
  }

  public void update(String name, Distance height, Angle angle) {
    if (!name.equals(currentState)) {
      currentState = name;
      elevatorTable.getEntry("State").setString(name);
      stateStart.reset();
      stateChangeTime = stateStart.get();
    }
    double time = stateStart.get() - stateChangeTime;
    SmartDashboard.putData("ElevatorWrist", elevatorWrist);
  }
}
