package frc.robot.simulation;

import static edu.wpi.first.math.Nat.N1;
import static edu.wpi.first.math.Nat.N2;

import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDevice.Direction;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Num;
import edu.wpi.first.math.StateSpaceUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.Discretization;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.RobotState;
import org.ejml.simple.SimpleMatrix;

/**
 * Code modified from here: https://github.com/narmstro2020/Notes
 *
 * <p>Thanks to Tyler Veness (calcmogul) and the discussion on Chief Delphi here
 * https://www.chiefdelphi.com/t/dcmotorsim-but-from-ks-kv-and-ka/440603/17
 */
public class MotorSim implements Runnable, AutoCloseable {

  private static final double dtSeconds = 0.020;

  public static final MotorSim createSimpleMotor(
      int deviceNumber,
      String simulatorDeviceType,
      double kS,
      double kV,
      double kA,
      double positionMeasurementStdDev,
      double velocityMeasurementStdDev,
      DCMotor dcMotor) {

    return new MotorSim(
        deviceNumber,
        simulatorDeviceType,
        kS,
        kV,
        kA,
        0,
        Double.NEGATIVE_INFINITY,
        Double.POSITIVE_INFINITY,
        positionMeasurementStdDev,
        velocityMeasurementStdDev,
        dcMotor,
        false,
        0.0);
  }

  public static final MotorSim createElevatorMotor(
      int deviceNumber,
      String simulatorDeviceType,
      double kS,
      double kV,
      double kA,
      double kG,
      double minimumRotations,
      double maximumRotations,
      double positionMeasurementStdDev,
      double velocityMeasurementStdDev,
      DCMotor dcMotor) {

    return new MotorSim(
        deviceNumber,
        simulatorDeviceType,
        kS,
        kV,
        kA,
        kG,
        minimumRotations,
        maximumRotations,
        positionMeasurementStdDev,
        velocityMeasurementStdDev,
        dcMotor,
        false,
        0.0);
  }

  public static final MotorSim createArmMotor(
      int deviceNumber,
      String simulatorDeviceType,
      double kS,
      double kV,
      double kA,
      double kG,
      double minimumRotations,
      double maximumRotations,
      double positionMeasurementStdDev,
      double velocityMeasurementStdDev,
      DCMotor dcMotor,
      double gearRatio) {

    return new MotorSim(
        deviceNumber,
        simulatorDeviceType,
        kS,
        kV,
        kA,
        kG,
        minimumRotations,
        maximumRotations,
        positionMeasurementStdDev,
        velocityMeasurementStdDev,
        dcMotor,
        true,
        gearRatio);
  }

  /** Continuous system matrix. */
  private final Matrix<N2, N2> m_A;

  /** Continuous input matrix. */
  private final Matrix<N2, N1> m_B;

  /** Output matrix. */
  private final Matrix<N2, N2> m_C;

  /** Feedthrough matrix. */
  private final Matrix<N2, N1> m_D;

  /** Static affine matrix */
  private final Matrix<N2, N1> m_c;

  /** Gravity matrix */
  private final Matrix<N2, N1> m_d;

  /** States Matrix */
  private final Matrix<N2, N1> m_x;

  /** Outputs Matrix */
  private final Matrix<N2, N1> m_y;

  /** Inputs Matrix */
  private final Matrix<N1, N1> m_u;

  /** Discretized Matrix A */
  private final Matrix<N2, N2> discA;

  /** Discretized Matrix B */
  private final Matrix<N2, N1> discB;

  private final boolean useMotorPosition;

  private final double gearRatio;

  /** The standard deviations of measurements, used for adding noise to the measurements. */
  private final Matrix<N2, N1> m_measurementStdDevs;

  /** Motor type used for calculations */
  private final DCMotor dcMotor;

  /** Minimum and Maximum Elevator Positions in Rotations */
  private final double minimumRotations;

  private final double maximumRotations;

  private final SimDevice simDevice;
  /** Motor Variables for Simulator Device Table */
  private final SimDouble simPosition;

  private final SimDouble simVelocity;
  private final SimDouble simVoltage;
  private final SimDouble simCurrent;

  /** Constructors */
  private MotorSim(
      int deviceNumber,
      String simulatorDeviceType,
      double kS,
      double kV,
      double kA,
      double kG,
      double minimumRotations,
      double maximumRotations,
      double positionMeasurementStdDev,
      double velocityMeasurementStdDev,
      DCMotor dcMotor,
      boolean useMotorPosition,
      double gearRatio) {

    Matrix<N2, N2> A = Matrix.mat(N2(), N2()).fill(0, 1, 0, -kV / kA);
    Matrix<N2, N1> B = Matrix.mat(N2(), N1()).fill(0, 1 / kA);
    Matrix<N2, N2> C = Matrix.eye(N2());
    Matrix<N2, N1> D = new Matrix<>(N2(), N1());
    Matrix<N2, N1> c = Matrix.mat(N2(), N1()).fill(0, -kS / kA);
    Matrix<N2, N1> d = Matrix.mat(N2(), N1()).fill(0, -kG / kA);

    checkMatrix(A, "A");
    checkMatrix(B, "B");
    checkMatrix(C, "C");
    checkMatrix(D, "D");
    checkMatrix(c, "c");
    checkMatrix(d, "d");

    this.m_A = A;
    this.m_B = B;
    this.m_C = C;
    this.m_D = D;
    this.m_c = c;
    this.m_d = d;

    m_x = new Matrix<>(new SimpleMatrix(A.getNumRows(), 1));
    m_u = new Matrix<>(new SimpleMatrix(B.getNumCols(), 1));
    m_y = new Matrix<>(new SimpleMatrix(C.getNumRows(), 1));

    this.m_measurementStdDevs =
        Matrix.mat(N2(), N1()).fill(positionMeasurementStdDev, velocityMeasurementStdDev);

    simDevice = SimDevice.create(simulatorDeviceType, deviceNumber);
    this.simPosition = simDevice.createDouble("Position", Direction.kBidir, 0.0);
    this.simVelocity = simDevice.createDouble("Velocity", Direction.kBidir, 0.0);
    this.simVoltage = simDevice.createDouble("Voltage", Direction.kBidir, 0.0);
    this.simCurrent = simDevice.createDouble("Current", Direction.kBidir, 0.0);
    this.dcMotor = dcMotor;
    this.minimumRotations = minimumRotations;
    this.maximumRotations = maximumRotations;

    var discABpair = Discretization.discretizeAB(m_A, m_B, dtSeconds);
    this.discA = discABpair.getFirst();
    this.discB = discABpair.getSecond();
    this.useMotorPosition = useMotorPosition;
    this.gearRatio = gearRatio;
  }

  // * Matrix Checker for Non Numbers */
  public <NA extends Num, NB extends Num> void checkMatrix(Matrix<NA, NB> matrix, String name) {
    for (int row = 0; row < matrix.getNumRows(); ++row) {
      for (int col = 0; col < matrix.getNumCols(); ++col) {
        if (!Double.isFinite(matrix.get(row, col))) {
          throw new IllegalArgumentException(
              String.format(
                  "Elements of %s aren't finite. This is usually due to model implementation errors.",
                  name));
        }
      }
    }
  }

  @Override
  public void run() {
    if (RobotState.isDisabled()) {
      this.m_u.set(0, 0, 0);
    }
    update();
    this.simVoltage.set(getInputVoltage());
    this.simCurrent.set(getCurrentDrawAmps());
    this.simPosition.set(getAngularPositionRot());
    this.simVelocity.set(getAngularVelocityRPM());
  }

  public double getAngularPositionRot() {
    return m_y.get(0, 0);
  }

  public double getAngularVelocityRPM() {
    return m_y.get(1, 0) * 60;
  }

  public void setAngularPosition(double rotations) {
    m_x.set(0, 0, rotations);
  }

  public double getCurrentDrawAmps() {
    double speedRadiansPerSec = 2 * Math.PI * getAngularVelocityRPM() * 1 / 60.0;
    double voltageInputVolts = m_u.get(0, 0);
    double voltageInputVoltsDirection = Math.signum(m_u.get(0, 0));
    return dcMotor.getCurrent(speedRadiansPerSec, voltageInputVolts) * voltageInputVoltsDirection;
  }

  public double getInputVoltage() {
    return this.m_u.get(0, 0);
  }

  public void setInputVoltage(double volts) {
    Matrix<N1, N1> u = Matrix.mat(N1(), N1()).fill(volts);
    Matrix<N1, N1> clampedU = clampInput(u);
    m_u.set(0, 0, clampedU.get(0, 0));
  }

  public void stop() {
    setInputVoltage(0.0);
  }

  private void update() {
    Matrix<N2, N1> x = calculateX();
    if (wouldHitLowerLimit(x.get(0, 0))) {
      x.set(0, 0, minimumRotations);
    }

    if (wouldHitUpperLimit(x.get(0, 0))) {
      x.set(0, 0, maximumRotations);
    }
    m_x.set(0, 0, x.get(0, 0));
    m_x.set(1, 0, x.get(1, 0));

    Matrix<N2, N1> y = calculateY();
    m_y.set(0, 0, y.get(0, 0));
    m_y.set(1, 0, y.get(1, 0));

    if (m_measurementStdDevs.get(0, 0) != 0 && m_measurementStdDevs.get(1, 0) != 0) {
      Matrix<N2, N1> newY = m_y.plus(StateSpaceUtil.makeWhiteNoiseVector(m_measurementStdDevs));
      m_y.set(0, 0, newY.get(0, 0));
      m_y.set(1, 0, newY.get(1, 0));
    }
  }

  private Matrix<N2, N1> calculateX() {
    Matrix<N1, N1> offSetMatrix =
        useMotorPosition
            ? VecBuilder.fill(Math.cos(m_x.get(0, 0) * 2 * Math.PI / gearRatio))
            : VecBuilder.fill(1.0);
    Matrix<N1, N1> bplusc =
        m_B.solve(m_c.times(Math.signum(m_x.get(1, 0))).plus(m_d.times(offSetMatrix)));
    Matrix<N2, N1> discc = discB.times(bplusc);
    return discA.times(m_x).plus(discB.times(m_u)).plus(discc);
  }

  private Matrix<N2, N1> calculateY() {
    return m_C.times(m_x).plus(m_D.times(m_u));
  }

  private Matrix<N1, N1> clampInput(Matrix<N1, N1> u) {
    return StateSpaceUtil.desaturateInputVector(u, RobotController.getBatteryVoltage());
  }

  private boolean wouldHitLowerLimit(double rotations) {
    return rotations <= this.minimumRotations;
  }

  private boolean wouldHitUpperLimit(double rotations) {
    return rotations >= this.maximumRotations;
  }

  @Override
  public void close() throws Exception {
    simDevice.close();
  }
}
