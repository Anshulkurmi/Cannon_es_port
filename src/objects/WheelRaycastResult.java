package objects;

import collision.RaycastResult;
import math.Vec3;

public class WheelRaycastResult extends RaycastResult {
    public double suspensionLength;
    public Vec3 directionWorld;
    public int groundObject;

    // Default constructor
    public WheelRaycastResult() {
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

    // Constructors and methods if needed
}
