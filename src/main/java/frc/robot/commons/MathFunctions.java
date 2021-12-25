package frc.robot.commons;


// Class containing basic math functions
public class MathFunctions {

    private MathFunctions() { } // Private constructor so the class cannot be initialized

    // Finds the closest point on a segment to a point not on the segment given the
    // segment start, segment end, position, and distance that the position is between the segment start and end
    public static Vec2 vectorToSegment2(Vec2 segmentStart, Vec2 segmentEnd, Vec2 position, double t) {
        return new Vec2(
            (1 - t) * segmentStart.x + t * segmentEnd.x - position.x,
            (1 - t) * segmentStart.y + t * segmentEnd.y - position.y
        );
    }
    
}
