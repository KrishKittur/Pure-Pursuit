package frc.robot.autonomus;

import edu.wpi.first.wpilibj.geometry.Translation2d;

// Class representing path points
public class SpeedPoint {
    
    public final Translation2d translation; // Stores the x and y coordinates of the point
    public final double velocity; // Stores the velocity at the given point

    // Constructs a point with the specified translation and velocity
    public SpeedPoint(Translation2d translation, double velocity) {
        this.translation = translation;
        this.velocity = velocity;
    }

    // Constructs a point with the specified x, y, and velocity
    public SpeedPoint(double x, double y, double velocity) {
        this(new Translation2d(x, y), velocity);
    }

    // Returns the x component of the Point
    public double getX() {
        return this.translation.getX();
    }

    // Returns the y component of the Point
    public double getY() {
        return this.translation.getY();
    }

}
