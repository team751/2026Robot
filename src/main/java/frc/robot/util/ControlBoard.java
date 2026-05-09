package frc.robot.util;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.lib.PS5Controller;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.JiggleCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.SimpleUnstuck;
import frc.robot.commands.SpitCommand;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.drive.SwerveConstants;
import frc.robot.subsystems.drive.SwerveSubsystem;
import frc.robot.subsystems.intake.ExtenderSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.simulation.MapSimSwerveTelemetry;
import frc.robot.subsystems.transfer.TransferSubsystem;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;

// import frc.robot.subsystems.climber.ClimberSubsystem;

// TODO: Clean up the controller bindings for this
// TODO: Fix intake commands

// TODO: shift button! \/\/\/
// controller.circleButton.and(controller.crossButton).whileTrue(
//   new StartEndCommand(() -> ShooterSubsystem.getInstance().requestShoot(), () ->
// ShooterSubsystem.getInstance().requestIdle())
// );
// we get basically double the buttons that we have now. could also help us test stuff without
// ruining previous keybinds
// could also be helpful during comp for re-zeroing components mid match if we need to (like intake
// explodes and dies and we need to align it again)

public class ControlBoard {
  private static ControlBoard instance;

  /* Controllers */
  private PS5Controller driver = null;
  private PS5Controller operator = null;
  private SwerveSubsystem drive = SwerveSubsystem.getInstance();
  private boolean preciseControl = false;
  private boolean autoAim = false;
  private boolean axisAlign = false;
  private double lastShotTime = 0.3;
  private double operatorOffset = 0;

  public boolean isBlue = false;

  private PIDController autoAimController = new PIDController(0.4, 0.0, 0.01);
  private PIDController axisAlignController =
      new PIDController(0.5, 0, 0.05); // , new Constraints(1, 1));

  private enum ControllerPreset {
    DRIVER(0),
    OPERATOR(1);

    private final int port;

    ControllerPreset(int port) {
      this.port = port;
    }

    public int port() {
      return port;
    }
  }

  /* Subsystems */
  private final Superstructure superstructure = Superstructure.getInstance();

  private final SwerveRequest.FieldCentric driveRequest =
      new SwerveRequest.FieldCentric()
          .withDeadband(SwerveConstants.maxSpeed * 0.05)
          .withRotationalDeadband(SwerveConstants.maxAngularSpeed * 0.1)
          .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage)
          .withSteerRequestType(SwerveModule.SteerRequestType.Position)
          .withDesaturateWheelSpeeds(true)
          .withForwardPerspective(SwerveRequest.ForwardPerspectiveValue.OperatorPerspective);

  private ControlBoard() {
    DriverStation.silenceJoystickConnectionWarning(true);
    autoAimController.enableContinuousInput(-180, 180);
    tryInit();
  }

  public void tryInit() {
    if (driver == null) {
      driver = new PS5Controller(ControllerPreset.DRIVER.port());
      configureBindings(ControllerPreset.DRIVER, driver);

      drive.setDefaultCommand(drive.applyRequest(this::getDriverRequest));
      if (Utils.isSimulation())
        drive.registerTelemetry(new MapSimSwerveTelemetry(SwerveConstants.maxSpeed)::telemeterize);
      System.out.println("Driver Initialized");
    }

    if (operator == null) {
      operator = new PS5Controller(ControllerPreset.OPERATOR.port());
      configureBindings(ControllerPreset.OPERATOR, operator);
      System.out.println("Operator Initialized");
    }
  }

  public void displayUI() {}

  public static ControlBoard getInstance() {
    if (instance == null) instance = new ControlBoard();
    return instance;
  }

  private void configureBindings(ControllerPreset preset, PS5Controller controller) {
    switch (preset) {
      case DRIVER -> configureDriverBindings(controller);
      case OPERATOR -> configureOperatorBindings(controller);
      default -> throw new IllegalStateException("Unexpected value: " + preset);
    }
  }

  private static double getAxisAlignAngle(double currentDegrees) {
    currentDegrees = currentDegrees % 360;
    if (currentDegrees > 180) currentDegrees -= 360;
    if (currentDegrees < -180) currentDegrees += 360;
    return Math.abs(currentDegrees) <= 90 ? 0.0 : 180.0;
  }

  /* Driver bindings */
  private void configureDriverBindings(PS5Controller controller) {
    /* Intake */
    controller.leftTrigger.whileTrue(
        new IntakeCommand(IntakeSubsystem.getInstance(), ExtenderSubsystem.getInstance()));

    controller.crossButton.whileTrue(
        new JiggleCommand(
            IntakeSubsystem.getInstance(),
            ExtenderSubsystem.getInstance(),
            TransferSubsystem.getInstance()));

    // controller.squareButton.onTrue(
    // new InstantCommand(() -> {
    //     if (LimelightConstants.MT_SWITCH_DISTANCE_METERS == 1.5) {
    //         LimelightConstants.MT_SWITCH_DISTANCE_METERS = 300.0;
    //     } else {
    //         LimelightConstants.MT_SWITCH_DISTANCE_METERS = 1.5;
    //     }
    // })
    // );

    controller.dUp.whileTrue(
        new InstantCommand(() -> ExtenderSubsystem.getInstance().requestExtend()));

    controller.dDown.whileTrue(
        new InstantCommand(() -> ExtenderSubsystem.getInstance().requestRetract()));

    controller.dDown.whileTrue(
        new StartEndCommand(
            () -> IntakeSubsystem.getInstance().requestIntake(),
            () -> IntakeSubsystem.getInstance().requestIdle()));

    controller.dLeft.whileTrue(new SimpleUnstuck(TransferSubsystem.getInstance()));

    /* Shooter */
    controller.rightTrigger.whileTrue(
        new ShootCommand(ShooterSubsystem.getInstance(), TransferSubsystem.getInstance()));
    controller.rightTrigger.onFalse(new InstantCommand(() -> operatorOffset = 0));
    controller.rightTrigger.whileTrue(
        new StartEndCommand(() -> autoAim = true, () -> autoAim = false)
            .withName("Auto Aim Toggle"));

    controller.rightTrigger.whileTrue(
        new JiggleCommand(
            IntakeSubsystem.getInstance(),
            ExtenderSubsystem.getInstance(),
            TransferSubsystem.getInstance()));

    controller.rightJoystickButton.whileTrue(
        new StartEndCommand(() -> axisAlign = true, () -> axisAlign = false)
            .withName("Axis Align Toggle"));

    controller.leftBumper.whileTrue(
        new SpitCommand(
            TransferSubsystem.getInstance(),
            IntakeSubsystem.getInstance(),
            ExtenderSubsystem.getInstance()));

    controller.leftJoystickButton.whileTrue(
        new StartEndCommand(() -> axisAlign = true, () -> axisAlign = false)
            .withName("Axis Align Toggle"));

    controller.circleButton.onTrue(new InstantCommand(() -> drive.setRobotRotationByAlliance()));

    controller.rightBumper.whileTrue(
        new StartEndCommand(() -> preciseControl = true, () -> preciseControl = false)
            .withName("Precise Control Toggle")); // Fight me owen

    /*Maplesim Shooter */
    controller.rightTrigger.whileTrue(
        Commands.run(
            () -> {
              // Check cooldown - must wait 0.25 seconds between shots
              double currentTime = Timer.getFPGATimestamp();
              if (currentTime - lastShotTime < 0.3) {
                return; // Still in cooldown period
              }

              // Attempt to obtain a game piece from the intake
              // Exit early if no balls available - prevents spawning projectiles when empty
              if (!SwerveSubsystem.simDrivetrain.mapleSimIntake.obtainGamePieceFromIntake()) {
                return;
              }

              // Only create and launch projectile if we successfully obtained a ball
              RebuiltFuelOnFly fuelOnFly =
                  new RebuiltFuelOnFly(
                      // Specify the position of the chassis when the fuel is launched
                      SwerveSubsystem.simDrivetrain
                          .mapleSimDrive
                          .getSimulatedDriveTrainPose()
                          .getTranslation(),
                      // Specify the translation of the shooter from the robot center (in the
                      // shooter's reference frame)
                      new Translation2d(0, 0),
                      // Specify the field-relative speed of the chassis, adding it to the initial
                      // velocity of the projectile
                      SwerveSubsystem.simDrivetrain.mapleSimDrive
                          .getDriveTrainSimulatedChassisSpeedsRobotRelative(),
                      // The shooter facing direction is the same as the robot's facing direction
                      SwerveSubsystem.simDrivetrain
                          .mapleSimDrive
                          .getSimulatedDriveTrainPose()
                          .getRotation(),
                      // Initial height of the flying fuel
                      Meters.of(1),
                      // The launch speed is proportional to the RPM; assumed to be 16 meters/second
                      // at 6000 RPM
                      MetersPerSecond.of(8.5),
                      // The angle at which the fuel is launched
                      Radians.of(1.4));

              SimulatedArena.getInstance().addGamePieceProjectile(fuelOnFly);

              // Update last shot time for cooldown tracking
              lastShotTime = currentTime;
            }));
  }

  /* Operator bindings */
  private void configureOperatorBindings(PS5Controller controller) {
    controller.rightTrigger.whileTrue(
        new ShootCommand(ShooterSubsystem.getInstance(), TransferSubsystem.getInstance(), true));

    controller.dUp.whileTrue(
        new InstantCommand(() -> ExtenderSubsystem.getInstance().requestExtend()));

    controller.dDown.whileTrue(
        new InstantCommand(() -> ExtenderSubsystem.getInstance().requestRetract()));

    controller.dLeft.whileTrue(new InstantCommand(() -> operatorOffset += isBlue ? -5.0 : 5.0));
    controller.dRight.whileTrue(new InstantCommand(() -> operatorOffset += isBlue ? 5.0 : -5.0));

    controller.crossButton.whileTrue(
        new JiggleCommand(
            IntakeSubsystem.getInstance(),
            ExtenderSubsystem.getInstance(),
            TransferSubsystem.getInstance()));
    controller.leftTrigger.whileTrue(
        new IntakeCommand(IntakeSubsystem.getInstance(), ExtenderSubsystem.getInstance()));

    controller.crossButton.whileTrue(
        new JiggleCommand(
            IntakeSubsystem.getInstance(),
            ExtenderSubsystem.getInstance(),
            TransferSubsystem.getInstance()));

    // controller.squareButton.onTrue(
    // new InstantCommand(() -> {
    //     if (LimelightConstants.MT_SWITCH_DISTANCE_METERS == 1.5) {
    //         LimelightConstants.MT_SWITCH_DISTANCE_METERS = 300.0;
    //     } else {
    //         LimelightConstants.MT_SWITCH_DISTANCE_METERS = 1.5;
    //     }
    // }));

    /* Climber */
    // TODO: Make left trigger shoot(peter requested)
    // controller.leftTrigger.whileTrue(
    //     new StartEndCommand(() -> climber.spinSlow(1), () -> climber.stopMotors()));

    // controller.crossButton.whileTrue(
    //     new StartEndCommand(() -> climber.spinSlow(-1), () -> climber.stopMotors()));

  }

  public SwerveRequest getDriverRequest() {
    if (driver == null) return null;

    double scale = preciseControl ? 0.2 : 1.0;
    double rotScale = preciseControl ? 0.30 : 1.0;

    double rawStickRot = driver.rightHorizontalJoystick.getAsDouble();
    double rot =
        rotScale
            * SwerveConstants.maxAngularSpeed
            * (Math.copySign(rawStickRot * rawStickRot, rawStickRot));

    if (autoAim) {
      Pose2d robotPose = drive.getPose();
      Pose2d hubPose = FieldConstants.getAllianceHub();
      double angleDiff =
          Math.toDegrees(
              Math.atan2(hubPose.getY() - robotPose.getY(), hubPose.getX() - robotPose.getX()));
      // SmartDashboard.putNumber("target offness", angleDiff -
      // robotPose.getRotation().getDegrees());
      rot =
          autoAimController.calculate(
              robotPose.getRotation().getDegrees(), angleDiff + operatorOffset);
      // SmartDashboard.putNumber("pid value", rot);
    }

    double x = driver.leftVerticalJoystick.getAsDouble();
    double y;

    if (axisAlign) {
      Pose2d robotPose = drive.getPose();

      Pose2d nearestTrench = new Pose2d(5, 5, Rotation2d.fromDegrees(0));
      Pose2d thing = FieldConstants.getNearestTrench(robotPose, isBlue);
      // SmartDashboard.putNumber("ControlBoard/paigus", thing.getY());
      y = (isBlue ? 1.0 : -1.0) * axisAlignController.calculate(robotPose.getY(), thing.getY());
      y = Math.min(y, 1.0);
      y = Math.max(y, -1.0);
      rot =
          autoAimController.calculate(
              robotPose.getRotation().getDegrees(),
              getAxisAlignAngle(robotPose.getRotation().getDegrees()));
    } else {
      y = driver.leftHorizontalJoystick.getAsDouble();
    }

    // SmartDashboard.putBoolean("ControlBoard/AutoAim", autoAim);
    // SmartDashboard.putBoolean("ControlBoard/AxisAlign", axisAlign);
    // SmartDashboard.putNumber("ControlBoard/OperatorOffset", operatorOffset);
    // SmartDashboard.putNumber("ControlBoard/limelightdist",
    // LimelightConstants.MT_SWITCH_DISTANCE_METERS);

    return driveRequest
        .withVelocityX(SwerveConstants.maxSpeed * x * scale)
        .withVelocityY(SwerveConstants.maxSpeed * y * scale)
        .withRotationalRate(rot);
  }
}
