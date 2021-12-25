package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.system.plant.DCMotor;
import edu.wpi.first.wpilibj.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import edu.wpi.first.wpiutil.math.VecBuilder;


// Contains the hardware and methods pertaining soley to the drive subsystem
public class Drive extends SubsystemBase {

    // Constants
    public static final double KvLinear = 1.98; // Feed forward kv linear
    public static final double KaLinear = 0.2; // Feed forward ka linear
    public static final double KvAngular = 1.5; // Feed forward kv angular
    public static final double KaAngular = 0.3; // Feed forward ka angular
    public static final double trackWidth = Units.inchesToMeters(20.0); // Robot's track width
    public static final double encoderResolution = 2048; // Resolution of our encoder
    public static final double wheelRadius = Units.inchesToMeters(3.0); // Radius of the wheels

    // Motors
    private final WPI_TalonFX leftMotor = new WPI_TalonFX(0); // Falcon that controls the left side of the drive train
    private final WPI_TalonFX rightMotor = new WPI_TalonFX(1); // Falcon that controls the right side of the drive train

    // Sensors
    AnalogGyro gyro = new AnalogGyro(0); // Analog gyro to measure the angle of the robot
    AnalogGyroSim gyroSim = new AnalogGyroSim(gyro); // Analog gyro sim for the robot
    Encoder leftEncoder = new Encoder(0, 1); // a/b channel encoder that measures the left side of the drive train
    Encoder rightEncoder = new Encoder(2, 3); // a/b channel encoder that measures the right side of the drive train
    EncoderSim leftEncoderSim = new EncoderSim(leftEncoder); // Encoder sim for the left side of the drive train
    EncoderSim rightEncoderSim = new EncoderSim(rightEncoder); // Encoder sim for the right side of the drive train

    // PID Controllers
    PIDController velocityPID = new PIDController(0.01, 0.0, 0.0);
    
    // Drive train sim
    DifferentialDrivetrainSim driveSim = new DifferentialDrivetrainSim(
        LinearSystemId.identifyDrivetrainSystem(KvLinear, KaLinear, KvAngular, KaAngular),
        DCMotor.getFalcon500(1),
        7.29, 
        0.7112,
        Units.inchesToMeters(3),
        VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005)
    );

    // Odometry
    private final DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(getAngle());
    private Pose2d pose = new Pose2d(0, 0, new Rotation2d());
    public final Field2d field = new Field2d();
 

    // Set encoder distance per pulses in constructor and update Smart Dashboard
    public Drive() {
        leftEncoder.setDistancePerPulse(2.0*Math.PI*wheelRadius/encoderResolution); // Set left encoder distance per pulse according to C=2*PI*r
        rightEncoder.setDistancePerPulse(2.0*Math.PI*wheelRadius/encoderResolution); // Set right encoder distance per pulse according to C=2*PI*r
        SmartDashboard.putData(field);
    }

    // Sets each motor to the desired percent output
    public void setPercent(double leftPercent, double rightPercent) {
        this.leftMotor.set(leftPercent);
        this.rightMotor.set(rightPercent);
    }

    // Sets each motor to the desired voltage output
    public void setVoltage(double leftVoltage, double rightVoltage) {
        this.leftMotor.setVoltage(leftVoltage);
        this.rightMotor.setVoltage(rightVoltage);
    }

    // Sets each motor to the desired speed
    public void setSpeeds(double leftMPS, double rightMPS) {
        double leftFF = KvLinear * leftMPS; // Calculate left FF output using by multiplying kv by the desired left-side velocity
        double rightFF = KvLinear * rightMPS; // Calculate right FF out put using by multiplying kv by the desired right-side velocity
        double leftPID = velocityPID.calculate(leftEncoder.getRate(), leftMPS); // Calculate left PID output by comparing the measurement to the desired output
        double rightPID = velocityPID.calculate(rightEncoder.getRate(), rightMPS); // Calculate right PID output by comparing the measurement to the desired output

        double leftVoltage = MathUtil.clamp(leftFF+leftPID, -12.0, 12.0); // Clamp the left output
        double rightVoltage = MathUtil.clamp(rightFF+rightPID, -12.0, 12.0); // Clamp the right output

        setVoltage(leftVoltage, rightVoltage); // Set each motor to its desired voltage

        SmartDashboard.putNumber("LeftR", leftEncoder.getRate());
        SmartDashboard.putNumber("LeftD", leftMPS);
    }

    // Returns the current gyro angle of the robot
    public Rotation2d getAngle() {
        return Rotation2d.fromDegrees(-gyro.getAngle());
    }

    // Returns the current pose of the robot
    public Pose2d getPose() {
        return pose;
    }

    // Sets the position of the robot
    public void setPose(Pose2d pose) {
        this.pose = pose;
        this.driveSim.setPose(pose);
    }

    // Update field position in the periodic method
    @Override
    public void periodic() {
        this.pose = odometry.update(getAngle(), leftEncoder.getDistance(), rightEncoder.getDistance()); // Updates the robot's position
        field.setRobotPose(pose);
    }
    
    // Handle the drivetrain simulation in the simulation periodic method
    @Override
    public void simulationPeriodic() {

        driveSim.setInputs(leftMotor.get()*RobotController.getBatteryVoltage(), rightMotor.get()*RobotController.getBatteryVoltage()); // Update the sim
        driveSim.update(0.02); // 20 millisecond dt

        // Updates the sensors according to the drive sim
        leftEncoderSim.setDistance(driveSim.getLeftPositionMeters());
        leftEncoderSim.setRate(driveSim.getLeftVelocityMetersPerSecond());
        rightEncoderSim.setDistance(driveSim.getRightPositionMeters());
        rightEncoderSim.setRate(driveSim.getRightVelocityMetersPerSecond());
        gyroSim.setAngle(-driveSim.getHeading().getDegrees());
        
    }

}
