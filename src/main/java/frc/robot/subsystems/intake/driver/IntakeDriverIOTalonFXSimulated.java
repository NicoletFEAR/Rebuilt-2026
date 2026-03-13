package frc.robot.subsystems.intake.driver;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.util.CANId;

public class IntakeDriverIOTalonFXSimulated extends IntakeDriverIO {
    private final TalonFX m_motor;
    private final DCMotorSim m_motorSim;

    public IntakeDriverIOTalonFXSimulated(CANId id) {
        m_motor = new TalonFX(id.getDevice(), id.getBus());
        m_motorSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), 0.001, 0.6), DCMotor.getKrakenX60(1));
    }

    @Override
    public void setVelocity(AngularVelocity velocity) {}

    @Override
    public void refreshData() {
        TalonFXSimState simState = m_motor.getSimState();
        simState.setSupplyVoltage(RobotController.getBatteryVoltage());

    }

    @Override
    public void updateInputs(IntakeDriverIOInputs inputs) {
        inputs.IntakeDriverType = Type.SIMULATED;
    }
}
