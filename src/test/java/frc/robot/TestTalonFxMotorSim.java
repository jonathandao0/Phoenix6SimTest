package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotEquals;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.robot.constants.SWERVE;
import frc.robot.simulation.MotorSim;
import frc.robot.utils.CtreUtils;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;

public class TestTalonFxMotorSim implements AutoCloseable {
  static final double DELTA = 1e-3; // acceptable deviation range
  static final double WAIT_TIME = 0.02;
  static final double MAX_SPEED_RPS = 6380.0 / 60.0;

  NetworkTableInstance m_nt;
  NetworkTable m_table;

  TalonFX m_driveMotor;
  TalonFXSimState m_driveMotorSimState;
  TalonFX m_turnMotor;
  TalonFXSimState m_turnMotorSimState;

  MotorSim m_driveMotorSim;
  MotorSim m_turnMotorSim;

  double m_currentTime = 0;
  double m_lastTime = 0;

  private void simulateMotorModels() {
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(
            m_driveMotorSim.getCurrentDrawAmps(), m_turnMotorSim.getCurrentDrawAmps()));
    m_driveMotorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
    m_turnMotorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
    m_driveMotorSim.setInputVoltage(
        MathUtil.clamp(m_driveMotorSimState.getMotorVoltage(), -12, 12));
    m_turnMotorSim.setInputVoltage(MathUtil.clamp(m_turnMotorSimState.getMotorVoltage(), -12, 12));

    m_currentTime = Timer.getFPGATimestamp();
    double dt = m_currentTime - m_lastTime;
    m_driveMotorSim.run();
    m_turnMotorSim.run();

    m_driveMotorSimState.setRawRotorPosition(
        m_driveMotorSim.getAngularPositionRot() * SWERVE.MODULE.kDriveMotorGearRatio);
    m_driveMotorSimState.setRotorVelocity(
        m_driveMotorSim.getAngularVelocityRPM() * SWERVE.MODULE.kDriveMotorGearRatio / 60.0);
    m_turnMotorSimState.setRawRotorPosition(
        m_turnMotorSim.getAngularPositionRot() * SWERVE.MODULE.kTurnMotorGearRatio);
    m_turnMotorSimState.setRotorVelocity(
        m_turnMotorSim.getAngularVelocityRPM() * SWERVE.MODULE.kTurnMotorGearRatio / 60.0);
    //    m_turnMotorSimState.setRawRotorPosition(m_turnMotorSim.getAngularPositionRot());
    //    m_driveMotorSimState.setRawRotorPosition(m_driveMotorSim.getAngularPositionRot());
    //    m_turnMotorSimState.setRotorVelocity(m_turnMotorSim.getAngularVelocityRPM() / 60.0);
    //    m_driveMotorSimState.setRotorVelocity(m_driveMotorSim.getAngularVelocityRPM() / 60.0);
    m_lastTime = m_currentTime;
  }

  @BeforeEach
  public void constructDevices() {
    assert HAL.initialize(500, 0);

    /* create the TalonFX */
    m_driveMotor = new TalonFX(0);
    var driveConfig = CtreUtils.generateDriveMotorConfig();
    driveConfig.Feedback.SensorToMechanismRatio = SWERVE.MODULE.kDriveMotorGearRatio;
    m_driveMotor.getConfigurator().apply(driveConfig);
    m_driveMotorSimState = m_driveMotor.getSimState();

    m_turnMotor = new TalonFX(1);
    var turnConfig = CtreUtils.generateTurnMotorConfig();
    turnConfig.Feedback.SensorToMechanismRatio = SWERVE.MODULE.kTurnMotorGearRatio;
    m_turnMotor.getConfigurator().apply(turnConfig);
    m_turnMotorSimState = m_turnMotor.getSimState();

    m_turnMotorSim =
        MotorSim.createSimpleMotor(
            m_turnMotor.getDeviceID(),
            "Falcon 500",
            0,
            0.135872794,
            0.002802309,
            0,
            0,
            DCMotor.getFalcon500(1));

    m_driveMotorSim =
        MotorSim.createSimpleMotor(
            m_driveMotor.getDeviceID(),
            "Falcon 500",
            0,
            0.134648227,
            0.002802309,
            0,
            0,
            DCMotor.getFalcon500(1));

    /* enable the robot */
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();

    RoboRioSim.resetData();

    m_nt = NetworkTableInstance.getDefault();
    m_nt.setServer("localhost", NetworkTableInstance.kDefaultPort4 + 1);
    m_nt.startClient4("unittest");
    m_table = m_nt.getTable("unittest");

    /* delay ~100ms so the devices can start up and enable */
    Timer.delay(0.200);
  }

  @AfterEach
  void shutdown() throws Exception {
    close();
  }

  @Test
  public void testVelocityControl() {
    var testSpeedMps = 4;
    var testSpeedRps = testSpeedMps / (SWERVE.MODULE.kWheelDiameterMeters * Math.PI);
    var velocityRequest = new VelocityVoltage(0);
    var velocitySignal = m_driveMotor.getVelocity();
    m_driveMotor.setControl(velocityRequest.withVelocity(testSpeedRps));
    Timer.delay(0.2);
    velocitySignal.waitForUpdate(0.1);

    var velPub = m_table.getDoubleTopic("velocity").publish();
    var batteryVPub = m_table.getDoubleTopic("batteryV").publish();
    var voltagePub = m_table.getDoubleTopic("voltage").publish();
    var currentPub = m_table.getDoubleTopic("current").publish();
    var idxPub = m_table.getIntegerTopic("idx").publish();
    var stopPub = false;
    var testVelRps = 0.0;
    var testVelMps = 0.0;
    m_lastTime = 0;
    for (int i = 0; i < 25; i++) {
      Timer.delay(WAIT_TIME);
      simulateMotorModels();
      velocitySignal.refresh();
      testVelRps = m_driveMotor.getVelocity().getValue();
      testVelMps = testVelRps * (Math.PI * SWERVE.MODULE.kWheelDiameterMeters);

      if (testVelMps > 4.4) {
        var test = m_driveMotor.getAppliedControl();
        var test1 = m_driveMotor.getClosedLoopError();
        var test2 = m_driveMotor.getClosedLoopReference();
      }

      velPub.set(testVelMps);
      voltagePub.set(m_driveMotorSimState.getMotorVoltage());
      currentPub.set(m_driveMotorSimState.getSupplyCurrent());
      batteryVPub.set(RobotController.getBatteryVoltage());
      if (testVelMps > testSpeedRps && !stopPub) {
        idxPub.set(i);
        stopPub = true;
      }
      m_nt.flush();
    }

    assertEquals(testSpeedMps, testVelMps, 0.2);
    velPub.close();
    idxPub.close();
  }

  @Test
  public void testPositionControl() {
    var testPositionDeg = 90.0;
    var testPositionRot = (testPositionDeg / 360.0) * SWERVE.MODULE.kTurnMotorGearRatio;
    var positionRequest = new PositionVoltage(0);
    var positionSignal = m_turnMotor.getPosition();
    m_turnMotor.setControl(positionRequest.withPosition(testPositionRot));
    Timer.delay(0.2);
    positionSignal.waitForUpdate(0.1);

    var posPub = m_table.getDoubleTopic("position").publish();
    var batteryVPub = m_table.getDoubleTopic("batteryV").publish();
    var voltagePub = m_table.getDoubleTopic("voltage").publish();
    var currentPub = m_table.getDoubleTopic("current").publish();
    var idxPub = m_table.getIntegerTopic("idx").publish();
    var stopPub = false;
    var testPosRot = 0.0;
    var testPosDeg = 0.0;
    m_lastTime = 0;
    for (int i = 0; i < 100; i++) {
      Timer.delay(WAIT_TIME);
      simulateMotorModels();
      positionSignal.refresh();
      testPosRot = positionSignal.getValue();
      testPosDeg = testPosRot * 360.0 / SWERVE.MODULE.kTurnMotorGearRatio;
      posPub.set(testPosDeg);
      voltagePub.set(m_turnMotorSimState.getMotorVoltage());
      currentPub.set(m_turnMotorSimState.getSupplyCurrent());
      batteryVPub.set(RobotController.getBatteryVoltage());
      if (m_turnMotor.getPosition().getValue() > testPositionRot && !stopPub) {
        idxPub.set(i);
        stopPub = true;
      }
      m_nt.flush();
    }

    assertEquals(testPositionDeg, testPosDeg, 0.1);
    posPub.close();
    idxPub.close();
  }

  @Disabled
  public void testPositionClean() {
    TalonFX m_testMotor = new TalonFX(2);
    var m_testMotorSim = m_testMotor.getSimState();
    var motorConfig = new TalonFXConfiguration();

    motorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    motorConfig.Slot0.kV = 0.0;
    motorConfig.Slot0.kP = 0.24;
    motorConfig.Slot0.kI = 0.0;
    motorConfig.Slot0.kD = 0.0;

    motorConfig.Voltage.PeakForwardVoltage = 12;
    motorConfig.Voltage.PeakReverseVoltage = -12;

    motorConfig.CurrentLimits.SupplyCurrentLimit = 25;
    motorConfig.CurrentLimits.SupplyCurrentThreshold = 40;
    motorConfig.CurrentLimits.SupplyTimeThreshold = 0.1;
    motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    m_testMotor.getConfigurator().apply(motorConfig);
    var testPositionRot = 100;
    var positionRequest = new PositionVoltage(0);
    var positionSignal = m_testMotor.getPosition();
    m_testMotor.setControl(positionRequest.withPosition(testPositionRot));
    Timer.delay(0.2);
    positionSignal.waitForUpdate(0.1);

    //        var m_turnMotorSimState =
    //                new FlywheelSim(
    //                        // Sim Values
    //                        LinearSystemId.identifyVelocitySystem(0.25, 0.000001),
    //                        DCMotor.getFalcon500(1),
    //                        1,
    //                        VecBuilder.fill(0));
    var m_turnMotorSim = new DCMotorSim(DCMotor.getFalcon500(1), 1, 0.15);

    var posPub = m_table.getDoubleTopic("position").publish();
    var voltagePub = m_table.getDoubleTopic("voltage").publish();
    var idxPub = m_table.getIntegerTopic("idx").publish();
    var stopPub = false;

    double pos = 0;
    for (int i = 0; i < 100; i++) {
      RoboRioSim.setVInVoltage(
          BatterySim.calculateDefaultBatteryLoadedVoltage(m_turnMotorSim.getCurrentDrawAmps()));
      m_testMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());
      m_turnMotorSim.setInputVoltage(MathUtil.clamp(m_testMotorSim.getMotorVoltage(), -12, 12));

      m_currentTime = Timer.getFPGATimestamp();
      double dt = m_currentTime - m_lastTime;
      m_turnMotorSim.update(dt);
      pos += m_turnMotorSim.getAngularVelocityRPM() / 60.0 * dt;
      m_testMotorSim.setRawRotorPosition(pos);
      m_testMotorSim.setRotorVelocity(m_turnMotorSim.getAngularVelocityRPM() / 60.0);

      m_lastTime = m_currentTime;

      positionSignal.refresh();
      voltagePub.set(m_testMotor.getBridgeOutput().getValueAsDouble());
      posPub.set(positionSignal.getValue());
      if (m_testMotor.getPosition().getValue() > testPositionRot && !stopPub) {
        idxPub.set(i);
        stopPub = true;
      }
      m_nt.flush();
      Timer.delay(WAIT_TIME);
    }

    var testCtrl = m_testMotor.getAppliedControl();
    assertEquals(m_testMotor.getPosition().getValue(), positionSignal.getValue(), 0.1);
    posPub.close();
    idxPub.close();
  }

  @Disabled
  public void testPositionSignal() {
    m_turnMotorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

    var posReq = new PositionVoltage(0);
    var posSig = m_turnMotor.getPosition();

    m_turnMotor.setControl(posReq.withPosition(5));
    Timer.delay(0.2);
    posSig.waitForUpdate(0.1);
    posSig.refresh();

    assertNotEquals(0.0, posSig.getValue());
  }

  @Disabled
  public void testVoltageSignal() {
    var dutyCycleSignal = m_driveMotor.getDutyCycle();

    m_driveMotor.set(1);
    Timer.delay(0.2);
    //        positionSignal.waitForUpdate(0.1);

    var voltagePub = m_table.getDoubleTopic("voltage").publish();
    for (int i = 0; i < 100; i++) {
      m_driveMotorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

      voltagePub.set(m_driveMotor.getBridgeOutput().getValueAsDouble());
    }

    assertEquals(12, m_driveMotor.getBridgeOutput().getValueAsDouble());
  }

  @Test
  public void testSimDevices() {
    assertEquals(12, BatterySim.calculateDefaultBatteryLoadedVoltage(0));
    RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(0));
    assertEquals(12, RobotController.getBatteryVoltage());
  }

  @AfterEach
  @Override
  public void close() throws Exception {
    /* destroy our TalonFX object */
    m_driveMotor.close();
    m_turnMotor.close();
    m_turnMotorSim.close();
    m_driveMotorSim.close();
    m_nt.close();
  }
}
