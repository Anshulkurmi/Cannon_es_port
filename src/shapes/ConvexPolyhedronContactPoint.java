package shapes;

import math.Vec3;

public class ConvexPolyhedronContactPoint {
    public Vec3 point;
    public Vec3 normal;
    public double depth;

    //added constructor 
    public ConvexPolyhedronContactPoint(){
        this(new Vec3(), new Vec3() , 0);
    }

    public ConvexPolyhedronContactPoint(Vec3 point, Vec3 normal, double depth) {
        this.point = point;
        this.normal = normal;
        this.depth = depth;
    }

    public Vec3 getPoint() {
        return point;
    }

    public void setPoint(Vec3 point) {
        this.point = point;
    }

    public Vec3 getNormal() {
        return normal;
    }

    public void setNormal(Vec3 normal) {
        this.normal = normal;
    }

    public double getDepth() {
        return depth;
    }

    public void setDepth(double depth) {
        this.depth = depth;
    }
}
