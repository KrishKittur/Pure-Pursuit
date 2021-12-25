package frc.robot.autonomus;


import java.util.LinkedHashSet;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import frc.robot.commons.MathFunctions;
import frc.robot.commons.Vec2;

// Path segment (equivelant of a linked list node)
public class PathSegment {

    public final SpeedPoint val; // Stores the value of the segment's starting point
    public PathSegment next; // Pointer to the segment's end point

    // Constructs a segment with the specified value
    public PathSegment(SpeedPoint val) {
        this.val = val;
        this.next = null;
    }

    // Returns whether this segment is the ending segment in the path
    public boolean isNullSegment() {
        return this.next == null;
    }

    // Returns the length of the segment
    public double getLength() {
        if (!isNullSegment()) 
            return getDelta().getNorm();
        return 0;
    }

    // Returns the number of intersections with a circle at a given position and with a given radius
    public LinkedHashSet<Translation2d> getIntersections(Translation2d position, double r) {

        LinkedHashSet<Translation2d> ans = new LinkedHashSet<>();

        if (!isNullSegment()) {
            Vec2 segmentStart = new Vec2(val.translation); 
            Vec2 segmentEnd = new Vec2(next.val.translation);
            Vec2 vectorPosition = new Vec2(position);

            Vec2 d = segmentEnd.minus(segmentStart); // Segment start to segment end
            Vec2 f = segmentStart.minus(vectorPosition); // Segment start to circle center

            double a = d.square(); // "a" coefficient in quadratic equation
            double b = 2 * f.dot(d); // "b" coefficient in quadratic equation
            double c = f.square() - r * r;

            double discriminant = b * b - 4 * a * c; // Discriminant of quadratic formula
            if (discriminant >= 0) { 
                discriminant = Math.sqrt(discriminant);

                // Either solution can be off the segment so we need to test them
                double t1 = (-b - discriminant)/(2 * a);
                double t2 = (-b + discriminant)/(2 * a);

                if (t1 >= 0 && t1 <= 1) // This means t1 is inside the segment
                    ans.add(MathFunctions.vectorToSegment2(segmentStart, segmentEnd, new Vec2(0, 0), t1).toTranslation2d());
                if (t2 >= 0 && t2 <= 1) // This means t2 is inside the segment
                    ans.add(MathFunctions.vectorToSegment2(segmentStart, segmentEnd, new Vec2(0, 0), t2).toTranslation2d());
            }
        }

        return ans;
    }

    // Returns the point on the segment closest to the robot
    public Translation2d getClosestPoint(Translation2d position) {
        if (!isNullSegment()) {
            Vec2 vectorPosition = new Vec2(position); // Vector representing the position of the robot
            Vec2 segmentStart = new Vec2(val.translation); // Vector representing the start of the current segment
            Vec2 segmentEnd = new Vec2(next.val.translation); // Vector representing the end of the current segment
            
            Vec2 v = segmentEnd.minus(segmentStart); // Delta segment start to end
            Vec2 u = segmentStart.minus(vectorPosition); // Delta segment start to position

            double t = -(v.dot(u)/v.dot(v)); // t represents the distance between the segment start and segment end position is

            if (t >= 0 && t <= 1) {
                return MathFunctions.vectorToSegment2(segmentStart, segmentEnd, new Vec2(0, 0), t).toTranslation2d();
            } else {
                double g0 = MathFunctions.vectorToSegment2(segmentStart, segmentEnd, vectorPosition, 0).square();
                double g1 = MathFunctions.vectorToSegment2(segmentStart, segmentEnd, vectorPosition, 1).square();
                return g0 <= g1 ? segmentStart.toTranslation2d() : segmentEnd.toTranslation2d();
            }
        } else {
            return this.val.translation;
        }
    }

    // Returns the remaining distance left in the segment from a point
    public double getRemainingDistance(Translation2d position) {
        if (!isNullSegment())
            return next.val.translation.minus(position).getNorm();
        else 
            return 0;
    }

    // Returns the distance the robot has travelled along the segment 
    public double getDistanceTravelled(Translation2d robotPosition) {
        Translation2d pathPosition = getClosestPoint(robotPosition);
        return getLength() - getRemainingDistance(pathPosition);
    }
    
    // Returns the percent the robot has travelled along the segment
    public double getPercentTravelled(Translation2d robotPosition) {
        return getDistanceTravelled(robotPosition) / getLength();
    }

    // Returns the point a specified distance along the segment
    public Translation2d getPointFromDistance(double dist) {
        if (!isNullSegment()) 
            return next.val.translation.minus(getDelta().times(dist / getLength()));
        else 
            return val.translation;
        
    }

    // Returns the change in position from the start of the segment to the end
    public Translation2d getDelta() {
        if (!isNullSegment()) {
            return next.val.translation.minus(val.translation);
        } else {
            return new Translation2d();
        }
    }

}
 