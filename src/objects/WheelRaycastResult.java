package objects;

import collision.RaycastResult;
import math.Vec3;

public class WheelRaycastResult extends RaycastResult {
    public double suspensionLength;
    public Vec3 directionWorld;
    public int groundObject;

    // Default constructor
    public WheelRaycastResult() {
    	super();
        this.suspensionLength = 0.0; // Default value
        this.directionWorld = new Vec3(); // Default value as a new Vec3
        this.groundObject = 0; // Default value
    }

    // Parameterized constructor
    public WheelRaycastResult(double suspensionLength, Vec3 directionWorld, int groundObject) {
        this.suspensionLength = suspensionLength;
        this.directionWorld = directionWorld;
        this.groundObject = groundObject;
    }
    

    // Getters and setters for suspensionLength
    public Double getSuspensionLength() {
        return suspensionLength;
    }

    public void setSuspensionLength(Double suspensionLength) {
        this.suspensionLength = suspensionLength;
    }

    // Getters and setters for directionWorld
    public Vec3 getDirectionWorld() {
        return directionWorld;
    }

    public void setDirectionWorld(Vec3 directionWorld) {
        this.directionWorld = directionWorld;
    }

    // Getters and setters for groundObject
    public Integer getGroundObject() {
        return groundObject;
    }

    public void setGroundObject(Integer groundObject) {
        this.groundObject = groundObject;
    }

    // Constructors and methods if needed
}
