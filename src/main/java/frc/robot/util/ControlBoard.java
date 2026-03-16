package frc.robot.util;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.lib.PS5Controller;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.JiggleCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.subsystems.Superstructure;
// import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.drive.SwerveConstants;
import frc.robot.subsystems.drive.SwerveSubsystem;
import frc.robot.subsystems.intake.ExtenderSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.simulation.MapSimSwerveTelemetry;
import frc.robot.subsystems.transfer.TransferSubsystem;

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

  private PIDController autoAimController = new PIDController(0.4, 0.0, 0.01);
  private ProfiledPIDController axisAlignController =
      new ProfiledPIDController(1.5, 0, 0.4, new Constraints(1, 1));

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

    controller.leftTrigger.whileTrue(
        new StartEndCommand(() -> preciseControl = true, () -> preciseControl = false)
            .withName("Precise Control Toggle")); // Fight me owen

    controller.crossButton.whileTrue(
        new JiggleCommand(
            IntakeSubsystem.getInstance(),
            ExtenderSubsystem.getInstance(),
            TransferSubsystem.getInstance(),
            ShooterSubsystem.getInstance()));
    controller.squareButton.whileTrue(
        new StartEndCommand(
            () -> IntakeSubsystem.getInstance().requestSpit(),
            () -> IntakeSubsystem.getInstance().requestIdle()));

    controller.dUp.whileTrue(
        new InstantCommand(() -> ExtenderSubsystem.getInstance().requestExtend()));

    controller.dDown.whileTrue(
        new InstantCommand(() -> ExtenderSubsystem.getInstance().requestRetract()));

    /* Shooter */
    controller.rightTrigger.whileTrue(
        new ShootCommand(ShooterSubsystem.getInstance(), TransferSubsystem.getInstance()));

    /* Transfer */
    controller.dLeft.whileTrue(
        new StartEndCommand(
            () -> TransferSubsystem.getInstance().requestReverse(),
            () -> TransferSubsystem.getInstance().requestIdle()));
    controller.dLeft.whileTrue(
        new StartEndCommand(
            () -> ShooterSubsystem.getInstance().requestSpit(),
            () -> ShooterSubsystem.getInstance().requestSpit()));

    controller.rightJoystickButton.whileTrue(
        new StartEndCommand(() -> axisAlign = true, () -> axisAlign = false)
            .withName("Axis Align Toggle"));

    controller.leftJoystickButton.whileTrue(
        new StartEndCommand(() -> axisAlign = true, () -> axisAlign = false)
            .withName("Axis Align Toggle"));

    controller.circleButton.onTrue(new InstantCommand(() -> drive.setRobotRotationByAlliance()));

    /* Swerve */
    controller.rightBumper.whileTrue(
        new StartEndCommand(() -> preciseControl = true, () -> preciseControl = false)
            .withName("Precise Control Toggle")); // Fight me owen
  }

  /* Operator bindings */
  private void configureOperatorBindings(PS5Controller controller) {
    controller.leftJoystickButton.onTrue(
        new InstantCommand(() -> drive.setRobotRotationByAlliance()));

    controller.squareButton.whileTrue(
        new StartEndCommand(() -> autoAim = true, () -> autoAim = false)
            .withName("Auto Aim Toggle"));

    controller.rightJoystickButton.whileTrue(
        new StartEndCommand(() -> axisAlign = true, () -> axisAlign = false)
            .withName("Axis Align Toggle"));

    /* Climber */
    // TODO: Make left trigger shoot(peter requested)
    // controller.leftTrigger.whileTrue(
    //     new StartEndCommand(() -> climber.spinSlow(1), () -> climber.stopMotors()));

    // controller.crossButton.whileTrue(
    //     new StartEndCommand(() -> climber.spinSlow(-1), () -> climber.stopMotors()));

  }

  public SwerveRequest getDriverRequest() {
    if (driver == null) return null;

    double scale = preciseControl ? 0.25 : 1.0;
    double rotScale = preciseControl ? 0.50 : 1.0;

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
      SmartDashboard.putNumber("target offness", angleDiff - robotPose.getRotation().getDegrees());
      rot = autoAimController.calculate(robotPose.getRotation().getDegrees(), angleDiff);
      SmartDashboard.putNumber("pid value", rot);
    }

    double x = driver.leftVerticalJoystick.getAsDouble();
    double y;

    if (axisAlign) {
      Pose2d robotPose = drive.getPose();
      Pose2d nearestTrench = FieldConstants.getNearestTrench(robotPose);
      y = -axisAlignController.calculate(robotPose.getY(), nearestTrench.getY());
      rot =
          autoAimController.calculate(
              robotPose.getRotation().getDegrees(),
              getAxisAlignAngle(robotPose.getRotation().getDegrees()));
    } else {
      y = driver.leftHorizontalJoystick.getAsDouble();
    }

    return driveRequest
        .withVelocityX(0.6 * SwerveConstants.maxSpeed * x * scale)
        .withVelocityY(0.6 * SwerveConstants.maxSpeed * y * scale)
        .withRotationalRate(rot);
  }
}
