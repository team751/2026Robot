package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.drive.SwerveSubsystem;
import frc.robot.util.ControlBoard;
import java.io.File;
import java.util.Arrays;
import java.util.Optional;
import java.util.stream.Stream;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.MethodSource;

/**
 * Picks each "Choreo: name" auto from RobotContainer's chooser and runs it in autonomous on the
 * CTRE swerve simulated by MapleSim, the same way the Sim GUI would. Reports how far the robot
 * strays from the path and where it stops. Runs in real time (a few seconds per trajectory). The
 * pose is MapleSim's ground truth, so this checks the follower, gains and Choreo robot config, not
 * vision.
 */
class ChoreoAutosTest {
  private static final double SETTLE = 1.0; // seconds to keep running after the trajectory ends

  private static RobotContainer container;
  private static SwerveSubsystem swerve;

  @BeforeAll
  static void setUp() throws InterruptedException {
    assertTrue(HAL.initialize(500, 0));
    // Same order as Robot(): swerve, then ControlBoard (sets the default drive command that
    // takes over and stops the robot when a trajectory ends), then RobotContainer
    swerve = SwerveSubsystem.getInstance();
    ControlBoard.getInstance();
    container = new RobotContainer();
    run(1.0); // let the CTRE sim devices come up
  }

  @AfterAll
  static void tearDown() {
    CommandScheduler.getInstance().cancelAll();
    DriverStationSim.setEnabled(false);
    DriverStationSim.notifyNewData();
  }

  static Stream<String> trajectories() {
    File[] files =
        new File(Filesystem.getDeployDirectory(), "choreo")
            .listFiles((dir, fileName) -> fileName.endsWith(".traj"));
    if (files == null) return Stream.empty();
    return Arrays.stream(files).map(f -> f.getName().replace(".traj", "")).sorted();
  }

  @ParameterizedTest(name = "{0}")
  @MethodSource("trajectories")
  void followsTrajectory(String name) throws InterruptedException {
    Optional<Trajectory<SwerveSample>> loaded = Choreo.loadTrajectory(name);
    assertTrue(loaded.isPresent(), name + ".traj did not load");
    Trajectory<SwerveSample> trajectory = loaded.get();
    assertTrue(trajectory.samples().size() > 1, name + ".traj has no samples; generate it");
    Pose2d end = trajectory.getFinalPose(false).orElseThrow();

    // Pick it in the chooser, like selecting it in the Sim GUI
    NetworkTable chooser =
        NetworkTableInstance.getDefault().getTable("SmartDashboard").getSubTable("Auto Chooser");
    chooser.getEntry("selected").setString("Choreo: " + name);
    // Twice: the chooser publishes "active" before it reads "selected", so it lags one update
    SmartDashboard.updateValues();
    SmartDashboard.updateValues();
    assertEquals("Choreo: " + name, chooser.getEntry("active").getString(""));

    // Enable autonomous and run it, the way Robot.autonomousInit does
    DriverStationSim.setAutonomous(true);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    Command auto = container.getAutonomousCommand();
    CommandScheduler.getInstance().schedule(auto);

    Pose2d trajStart = trajectory.getInitialPose(false).orElseThrow();
    double start = Timer.getFPGATimestamp();
    double maxOffPath = 0;
    double reachedEnd = Double.NaN;
    boolean onPath = false; // false until the sim-only odometry reset puts the robot at the start
    int loop = 0;
    while (Timer.getFPGATimestamp() - start < trajectory.getTotalTime() + SETTLE) {
      step();
      double t = Timer.getFPGATimestamp() - start;
      Pose2d pose = swerve.getPose();
      onPath |= pose.getTranslation().getDistance(trajStart.getTranslation()) < 0.10;
      if (onPath) maxOffPath = Math.max(maxOffPath, distanceToPath(trajectory, pose));
      if (Double.isNaN(reachedEnd)
          && pose.getTranslation().getDistance(end.getTranslation()) < 0.05) reachedEnd = t;
      if (loop++ % 5 == 0) { // speed trace every 0.1 s
        var speeds = swerve.getChassisSpeeds();
        System.out.printf(
            "  t=%4.2f  x=%6.3f  y=%6.3f  speed=%5.2f m/s%n",
            t,
            pose.getX(),
            pose.getY(),
            Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond));
      }
    }

    Pose2d pose = swerve.getPose();
    double positionError = pose.getTranslation().getDistance(end.getTranslation());
    double headingError = Math.abs(pose.getRotation().minus(end.getRotation()).getDegrees());
    System.out.printf(
        "%-20s planned %4.2f s, within 5 cm of end at %4.2f s | max off path %4.1f cm | "
            + "stopped at (%.3f, %.3f, %.1f deg), %4.1f cm / %4.1f deg from end%n",
        name,
        trajectory.getTotalTime(),
        reachedEnd,
        maxOffPath * 100,
        pose.getX(),
        pose.getY(),
        pose.getRotation().getDegrees(),
        positionError * 100,
        headingError);

    // Disabling ends the routine, the same as the end of auto
    DriverStationSim.setEnabled(false);
    DriverStationSim.notifyNewData();
    run(0.5);

    assertTrue(positionError < 0.05, name + " stopped " + positionError + " m from the end");
    assertTrue(headingError < 3, name + " stopped " + headingError + " deg off the end heading");
  }

  private static void run(double seconds) throws InterruptedException {
    double start = Timer.getFPGATimestamp();
    while (Timer.getFPGATimestamp() - start < seconds) step();
  }

  /** One robot loop: what TimedRobot does every 20 ms. */
  private static void step() throws InterruptedException {
    DriverStationSim.notifyNewData();
    SmartDashboard.updateValues();
    CommandScheduler.getInstance().run();
    Thread.sleep(20);
  }

  /** Closest distance from the pose to the trajectory's path, sampled finely along it. */
  private static double distanceToPath(Trajectory<SwerveSample> trajectory, Pose2d pose) {
    double best = Double.POSITIVE_INFINITY;
    for (int i = 0; i <= 400; i++) {
      Pose2d sample =
          trajectory.sampleAt(trajectory.getTotalTime() * i / 400.0, false).orElseThrow().getPose();
      best = Math.min(best, sample.getTranslation().getDistance(pose.getTranslation()));
    }
    return best;
  }
}
