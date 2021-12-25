package frc.robot.autonomus;

import java.util.LinkedHashSet;
import java.util.List;

import edu.wpi.first.wpilibj.geometry.Translation2d;
import static frc.robot.Constants.*;


// Represents the robot's autonomus path
// Implemented as a linked list
public class Path {

    // Represents the first segment of the path
    private PathSegment head;

    // Constructs a new path given start, interior, and end points
    public Path(SpeedPoint start, List<SpeedPoint> interiorPoints, SpeedPoint end) {
        head = new PathSegment(start);
        for (SpeedPoint sp : interiorPoints) 
            insertSegment(sp);
        insertSegment(end);
    }
    
    // Returns the target point given a specified lookahead distance and the robot position
    public SpeedPoint getTargetPoint(Translation2d robot, double lookahead) {

        PathSegment currentSegment = getFirst(); // Current segment
        Translation2d closestPoint = currentSegment.getClosestPoint(robot); // Closest on the current segment to the robot
        double fractionalProgress = currentSegment.getPercentTravelled(closestPoint); // Percent of the segment the robot has traveled

        Translation2d targetPointTranslation = null; // Variable to keep track of the target point translation
        int i = 0; // Variable to keep track of segment
        while (currentSegment.next != null && targetPointTranslation == null) {
            LinkedHashSet<Translation2d> intersections = currentSegment.getIntersections(robot, lookahead); // Get all the segment intersections given the robot position and lookahead distance
            for (Translation2d intersection : intersections) {
                double fractionalDistance = currentSegment.getPercentTravelled(intersection) + i;
                if (fractionalDistance > fractionalProgress) {
                    targetPointTranslation = intersection; 
                    break;
                }
            }
            currentSegment = currentSegment.next; // Increment current segment
            i++; // Increment counter variable
        }        

        if (targetPointTranslation == null) { // If we did not find an intersection point ahead of the robot
            targetPointTranslation = getFirst().next.val.translation; // Set the translation to the endpiont of the current segment
        }

        checkSegmentDone(robot); // At the end of the method check whether the current segment has been completed

        return new SpeedPoint(targetPointTranslation, getFirst().val.velocity); // Return the target point

    }

    // Method to insert a new segment 
    public void insertSegment(SpeedPoint newSegmentEndPoint) {
        PathSegment newSegment = new PathSegment(newSegmentEndPoint);
        PathSegment last = head;
        while (last.next != null) 
            last = last.next;
        last.next = newSegment;
    }

    // Returns the first element in the path
    public PathSegment getFirst() {
        return head;
    }

    // Returns the last element in the path
    public PathSegment getLast() {
        PathSegment last = head;
        while (last.next != null) 
            last = head.next;
        return last;
    }

    // Returns and removes the first segment from the path
    public PathSegment removeFirst() {
        PathSegment temp = head;
        head = head.next;
        return temp;
    }

    // Returns and removes the last segment from the path
    public PathSegment removeLast() {
        if (head == null) 
            return null;
        if (head.next == null)
            return null;

        PathSegment secondLast = head;
        while (secondLast.next.next != null) 
            secondLast = secondLast.next;

        PathSegment temp = secondLast.next;
        secondLast.next = null;
        return temp;
    }

    // Returns the number of segments in the path
    public int getNumSegments() {
        PathSegment last = head;
        int ans = 0;
        while (last.next != null) {
            ans++;
            last = last.next;
        }
        return ans;
    }

    // Checks whether the current segment is done and removes it from the path
    public void checkSegmentDone(Translation2d robotPose) {
        if (!isFinished()) {
            PathSegment current = head;
            double remainingDistance = current.getRemainingDistance(current.getClosestPoint(robotPose));
            if (remainingDistance < SEGMENT_COMPLETION_TOLERANCE)
                removeFirst();
        }
    }

    // Checks whether the path is finished
    public boolean isFinished() {
        return getNumSegments() == 0;
    }

}
