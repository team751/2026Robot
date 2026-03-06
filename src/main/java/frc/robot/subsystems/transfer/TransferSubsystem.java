package frc.robot.subsystems.transfer;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TransferSubsystem extends SubsystemBase {
  private static TransferSubsystem instance;

  // Motors
  private final TalonFX TransferTopMotor =
      TransferConstants.transfertopconfig.createDevice(TalonFX::new);
  private final VoltageOut transferTopControl = new VoltageOut(0);

  private final TalonFX TransferBottomMotor =
      TransferConstants.transferbottomconfig.createDevice(TalonFX::new);
  private final VoltageOut transferBottomControl = new VoltageOut(0);

  private void setTransferTopMotor(double voltage) {
    TransferTopMotor.setControl(transferTopControl.withOutput(voltage));
  }

  private void setTransferBottomMotor(double voltage) {
    TransferBottomMotor.setControl(transferBottomControl.withOutput(voltage));
  }
}
