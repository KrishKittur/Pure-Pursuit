package frc.robot;

import java.util.List;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.autonomus.Path;
import frc.robot.autonomus.PurePursuitController;
import frc.robot.autonomus.SpeedPoint;
import frc.robot.subsystems.Drive;

public class Robot extends TimedRobot {

  Drive drive = new Drive();
  XboxController controller = new XboxController(0);

  Path path = new Path(
    new SpeedPoint(new Translation2d(0, 0), 0.5),
    List.of(
      new SpeedPoint(new Translation2d(2, 3), 0.5),
      new SpeedPoint(new Translation2d(0, 4), 0.5)
    ),
    new SpeedPoint(new Translation2d(6, 4), 0.5)
  );

  PurePursuitController pp = new PurePursuitController(path, 1, Units.inchesToMeters(20.0));

  @Override
  public void robotInit() {
    drive.setDefaultCommand(
      new RunCommand(() -> {
        drive.setSpeeds(Math.abs(controller.getY(Hand.kLeft)) > 0.1 ? -controller.getY(Hand.kLeft) * 12.0/Drive.KvLinear : 0.0, Math.abs(controller.getY(Hand.kRight)) > 0.1 ? -controller.getY(Hand.kRight) * 12.0/Drive.KvLinear : 0.0);
      }, drive)
    );
    drive.setPose(new Pose2d(0, 0, Rotation2d.fromDegrees(56.3099325)));
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void autonomousInit() {
  }

  @Override
  public void autonomousPeriodic() {
    double[] speeds = pp.update(drive.getPose());
    drive.setSpeeds(speeds[0], speeds[1]);
  }

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {}

  @Override
  public void disabledInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}
}
