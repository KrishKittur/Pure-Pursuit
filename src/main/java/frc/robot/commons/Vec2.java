package frc.robot.commons;

import edu.wpi.first.wpilibj.geometry.Translation2d;

// Class for basic 2-dimensional vector operations
public class Vec2 {

    public final double x, y; // Stores the x and y components of the vector

    // Constructs a vector with the specified x and y components
    public Vec2(double x, double y) {
        this.x = x;
        this.y = y;
    }

    // Constructs a vector with the given Translation2d object
    public Vec2(Translation2d translation) {
        this(translation.getX(), translation.getY());
    }

    // Returns the given vector multiplied by a specified scalar
    public Vec2 scale(double scalar) {
        return new Vec2(this.x*scalar, this.y*scalar);
    }

    // Returns the some of two vectors
    public Vec2 add(Vec2 other) {
        return new Vec2(this.x+other.x, this.y+other.y);
    }

    // Returns the difference of two vectors
    public Vec2 minus(Vec2 other) {
        return new Vec2(this.x-other.x, this.y-other.y);
    }

    // Returns the dot product of two vectors
    public double dot(Vec2 other) {
        return this.x*other.x+this.y*other.y;
    }

    // Returns the cross product of two vectors 
    public double cross(Vec2 other) {
        return (this.y*other.x)-(this.x*other.y);
    }

    // Returns the dot product of this vector with itself
    public double square() {
        return this.dot(this);
    }

    // Returns the conversion of the vector to a Translation2d object
    public Translation2d toTranslation2d() {
        return new Translation2d(this.x, this.y);
    }
    
}
