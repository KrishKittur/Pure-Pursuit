package frc.robot.autonomus;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;

// This class represents the pure pursuit controller that should be used to follow an autonomus path
public class PurePursuitController {

    private final Path path; // Path variable to store the robot's autonomus path
    private final double lookahead, trackWidth; // Lookahead and robot trackwidth 

    // Consructs a PurePursuitController object with the specified path and lookahead distance
    public PurePursuitController(Path path, double lookahead, double trackWidth) {
        this.path = path;
        this.lookahead = lookahead;
        this.trackWidth = trackWidth;

    }

    // This method should be called periodically in the execute() method of a command or the autonomusPeriodic() method of robot.java
    // Returns a double array: [left-wheel-velocity, right-wheel-velocity]
    public double[] update(Pose2d robotPose) {
        if (isFinished()) // If the path is finished then the robot doesn't move
            return new double[] {0, 0};
        SpeedPoint target = path.getTargetPoint(robotPose.getTranslation(), lookahead);
        double curvature = getCurvature(robotPose, target.translation);
        System.out.println(path.getFirst().val.translation);
        return getSpeeds(curvature, target.velocity);
    }

    // Returns the curvature of the arc that passes through the robot's position and and a target point
    public double getCurvature(Pose2d robot, Translation2d target) {
        Pose2d targetPose = new Pose2d(target, Rotation2d.fromDegrees(0.0));
        double x = targetPose.relativeTo(robot).getY(); // Gets the x offset to the target point
        double distance = target.getDistance(robot.getTranslation()); // Distance between robot and target point
        return -(2 * x)/(distance * distance); // curvature = 2x/l^2
    }

    // Returns a double array: [left-wheel-velocity, right-wheel-velocity] with the specified curvature and velocity
    public double[] getSpeeds(double curvature, double velocity) {
        double L = velocity * (2 + curvature * trackWidth)/2;
        double R = velocity * (2 - curvature * trackWidth)/2;
        return new double[] {L, R};
    }

    // Returns whether or not the robot is at the end of its path
    public boolean isFinished() {
        return path.isFinished();
    }
    
}
