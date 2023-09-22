package shapes;

import math.Vec3; // Import the appropriate Vec3 class
import math.Quaternion; // Import the appropriate Quaternion class

/**
 * A plane, facing in the Z direction. The plane has its surface at z=0 and everything below z=0 is assumed to be solid plane. To make the plane face in some other direction than z, you must put it inside a Body and rotate that body. See the demos.
 * @example
 *     const planeShape = new CANNON.Plane()
 *     const planeBody = new CANNON.Body({ mass: 0, shape:  planeShape })
 *     planeBody.quaternion.setFromEuler(-Math.PI / 2, 0, 0) // make it face up
 *     world.addBody(planeBody)
 */
public class Plane extends Shape {
    public Vec3 worldNormal;
    public boolean worldNormalNeedsUpdate;
    private double boundingSphereRadius;

    public Plane() {
        super(ShapeTypes.PLANE); 

        // World oriented normal
        worldNormal = new Vec3();
        worldNormalNeedsUpdate = true;

        boundingSphereRadius = Double.MAX_VALUE;
    }

    /**
     * Compute the world normal of the plane based on the provided quaternion.
     * @param quat The quaternion to use for rotation.
     */
    public void computeWorldNormal(Quaternion quat) {
        Vec3 n = worldNormal;
        n.set(0, 0, 1);
        quat.vmult(n, n);
        worldNormalNeedsUpdate = false;
    }

    //changed nothing .check return type of calculateLocalInertia
    @Override
    public Vec3 calculateLocalInertia(double mass, Vec3 target) {
        // The plane has no mass, so the local inertia is zero.
        target.set(0, 0, 0);
        return target;
    }

    @Override
    public double volume() {
        // The plane is considered infinite, so its volume is set to a large positive value.
        return Double.MAX_VALUE;
    }

    @Override
    public void calculateWorldAABB(Vec3 pos, Quaternion quat, Vec3 min, Vec3 max) {
        // The plane AABB is infinite, except if the normal is pointing along any axis
        Vec3 tempNormal = new Vec3(0, 0, 1); // Default plane normal is z
        quat.vmult(tempNormal, tempNormal);
        double maxVal = Double.MAX_VALUE;
        min.set(-maxVal, -maxVal, -maxVal);
        max.set(maxVal, maxVal, maxVal);

        if (tempNormal.x == 1) {
            max.x = pos.x;
        } else if (tempNormal.x == -1) {
            min.x = pos.x;
        }

        if (tempNormal.y == 1) {
            max.y = pos.y;
        } else if (tempNormal.y == -1) {
            min.y = pos.y;
        }

        if (tempNormal.z == 1) {
            max.z = pos.z;
        } else if (tempNormal.z == -1) {
            min.z = pos.z;
        }
    }

    @Override
    public void updateBoundingSphereRadius() {
        // The plane's bounding sphere radius is set to a large positive value.
        boundingSphereRadius = Double.MAX_VALUE;
    }
}
