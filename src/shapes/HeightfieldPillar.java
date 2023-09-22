package shapes;

import math.Vec3;

public class HeightfieldPillar {
    public ConvexPolyhedron convex;
    public Vec3 offset;

    public HeightfieldPillar(ConvexPolyhedron convex, Vec3 offset) {
        this.convex = convex;
        this.offset = offset;
    }
}