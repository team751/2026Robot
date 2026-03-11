package frc.robot.subsystems;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.drive.SwerveSubsystem;
import frc.robot.subsystems.intake.ExtenderSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

// import frc.robot.subsystems.intake.IntakeSubsystem;

public class Superstructure extends SubsystemBase {
  private static Superstructure instance = null;

  enum SuperstructureState {
    PRE_HOME,
    IDLE,
  }

  private final SwerveSubsystem swerveSubsystem = SwerveSubsystem.getInstance();
  private final ShooterSubsystem shooterSubsystem = ShooterSubsystem.getInstance();
 // private final ClimberSubsystem climberSubsystem = ClimberSubsystem.getInstance();
  private final IntakeSubsystem intakeSubsystem = IntakeSubsystem.getInstance();
  private final ExtenderSubsystem extenderSubsystem = ExtenderSubsystem.getInstance();

  boolean requestHome = false;
  boolean requestIdle = false;

  double mStateStartTime = 0.0;
  private SuperstructureState systemState = SuperstructureState.PRE_HOME;

  boolean homedOnce = false;
  private double lastFPGATimestamp = 0.0;

  private Superstructure() {}

  public static Superstructure getInstance() {
    if (instance == null) instance = new Superstructure();
    return instance;
  }

  @Override
  public void periodic() {
    double time = RobotController.getFPGATime();
    SmartDashboard.putNumber("Superstructure/loopCycleTime", time - lastFPGATimestamp);
    lastFPGATimestamp = time;

    SuperstructureState nextState = systemState;
    switch (systemState) {
      case PRE_HOME -> {}
      case IDLE -> {}
      default -> throw new IllegalArgumentException("wops");
    }

    if (nextState != systemState) {
      mStateStartTime = time;
      systemState = nextState;
    }
  }

  public void unsetAllRequests() {
    requestHome = false;
    requestIdle = false;
  }

  public void requestHome() {
    unsetAllRequests();
    requestHome = true;
  }

  public void requestIdle() {
    unsetAllRequests();
    requestIdle = true;
  }
}
