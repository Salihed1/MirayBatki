package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants;

import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class Arm implements AutoCloseable {
  private double m_armKp = Constants.kDefaultArmKp;
  private double m_armSetpointDegrees = Constants.kDefaultArmSetpointDegrees * 1.5;

  private final DCMotor m_armGearbox = DCMotor.getVex775Pro(1);
  //kp, ki, kd random. Simulation didn't work!!!!!!!!!
  private final PIDController m_controller = new PIDController(m_armKp, 0, 0);
  private final Encoder m_encoder =
      new Encoder(Constants.kEncoderAChannel, Constants.kEncoderBChannel);
  private final TalonFX m_motor = new TalonFX(Constants.kMotorPort);

  private final SingleJointedArmSim m_armSim = new SingleJointedArmSim(m_armGearbox,Constants.kArmReduction,SingleJointedArmSim.estimateMOI(Constants.kArmLength, Constants.kArmMass),Constants.kArmLength,Constants.kMinAngleRads,Constants.kMaxAngleRads,true,VecBuilder.fill(Constants.kArmEncoderDistPerPulse));
  private final EncoderSim m_encoderSim = new EncoderSim(m_encoder);
  //Create a arm
  private final Mechanism2d m_mech2d = new Mechanism2d(60, 60);
  private final MechanismRoot2d m_armPivot = m_mech2d.getRoot("ArmPivot", 30, 30);
  private final MechanismLigament2d m_armTower =
      m_armPivot.append(new MechanismLigament2d("ArmTower", 30, -90));
  private final MechanismLigament2d m_arm = m_armPivot.append(new MechanismLigament2d("Arm",30,Units.radiansToDegrees(m_armSim.getAngleRads()),6,new Color8Bit(Color.kYellow)));

  public Arm() {
    m_encoder.setDistancePerPulse(Constants.kArmEncoderDistPerPulse);

    //Put Mechanism 2d to SmartDashboard
    SmartDashboard.putData("Arm Sim", m_mech2d);
    m_armTower.setColor(new Color8Bit(Color.kBlue));

    Preferences.initDouble(Constants.kArmPositionKey, m_armSetpointDegrees);
    Preferences.initDouble(Constants.kArmPKey, m_armKp);
    
    m_motor.configFactoryDefault();
    m_motor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
    m_motor.setSensorPhase(false);
    m_motor.setInverted(false);
  }

  //Voltage
  public void simulationPeriodic() {
    m_armSim.setInput(m_motor.getMotorOutputPercent() * RobotController.getBatteryVoltage());
    m_armSim.update(0.020);
    m_encoderSim.setDistance(m_armSim.getAngleRads());
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(m_armSim.getCurrentDrawAmps()));
    m_arm.setAngle(Units.radiansToDegrees(m_armSim.getAngleRads()));
  }

  /** Load setpoint and kP from preferences. */
  public void loadPreferences() {
    m_armSetpointDegrees = Preferences.getDouble(Constants.kArmPositionKey, m_armSetpointDegrees);
    if (m_armKp != Preferences.getDouble(Constants.kArmPKey, m_armKp)) {
      m_armKp = Preferences.getDouble(Constants.kArmPKey, m_armKp);
      m_controller.setP(m_armKp);
    }
  }

  public void reachSetpoint() {
    double setpointRadians = Units.degreesToRadians(m_armSetpointDegrees);
    double pidOutput = m_controller.calculate(m_encoder.getDistance(), setpointRadians);
    m_motor.set(ControlMode.PercentOutput, pidOutput);
  }

  public void stop() {
    m_motor.set(ControlMode.PercentOutput, 0.0);
  }

  @Override
  public void close() {
    //TalonFX doesn't have a close() method
    m_motor.set(ControlMode.PercentOutput, 0.0);
    m_motor.configSelectedFeedbackSensor(FeedbackDevice.None, 0, 0);
    m_motor.configFactoryDefault();

    m_encoder.close();
    m_mech2d.close();
    m_armPivot.close();
    m_controller.close();
    m_arm.close();
  }
}
