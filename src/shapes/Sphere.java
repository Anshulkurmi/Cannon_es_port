package shapes;

import math.Vec3;
import math.Quaternion;

/**
 * Spherical shape
 * @example
 *     final double radius = 1.0;
 *     final Sphere sphereShape = new Sphere(radius);
 *     final Body sphereBody = new Body(1.0, sphereShape); // Mass of 1.0
 *     world.addBody(sphereBody);
 */
public class Sphere extends Shape {
    /**
     * The radius of the sphere.
     */
    public double radius;
    
    
    public Sphere(double radius) {
    	this(radius,null);
    }

    /**
     *
     * @param radius The radius of the sphere, a non-negative number.
     */
    public Sphere(double radius, ShapeOptions options) {
        super(ShapeTypes.SPHERE,options);

        this.radius = (radius >= 0) ? radius : 1.0;

        if (this.radius < 0) {
            throw new IllegalArgumentException("The sphere radius cannot be negative.");
        }

        updateBoundingSphereRadius();
    }

    /** calculateLocalInertia */
    public Vec3 calculateLocalInertia(double mass, Vec3 target) {
        final double I = (2.0 * mass * radius * radius) / 5.0;
        target.set(I, I, I);
        return target;
    }

    /** volume */
    public double volume() {
        return (4.0 * Math.PI * Math.pow(radius, 3)) / 3.0;
    }

    public void updateBoundingSphereRadius() {
        this.boundingSphereRadius = radius;
    }

    public void calculateWorldAABB(Vec3 pos, Quaternion quat, Vec3 min, Vec3 max) {
        final double r = radius;
        //final String[] axes = {"x", "y", "z"};
        // for (String ax : axes) {
        //     min.set(ax, pos.get(ax) - r);
        //     max.set(ax, pos.get(ax) + r);
        // }
            //changed min and max 
        min.set(pos.x-r,pos.y-r,pos.z-r);
        max.set(pos.x+r,pos.y+r,pos.z+r);
    }
}
