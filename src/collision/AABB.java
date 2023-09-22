package collision;

import math.Vec3;

import java.util.Arrays;
import java.util.List;

import math.Transform;
import math.Quaternion;

/**
 * Axis aligned bounding box class.
 */
public class AABB {
    public Vec3 lowerBound; // The lower bound of the bounding box
    public Vec3 upperBound; // The upper bound of the bounding box
    
    public static Vec3 tmp = new Vec3();

    public AABB() {
        this.lowerBound = new Vec3();
        this.upperBound = new Vec3();
    }

    public AABB(Vec3 lowerBound, Vec3 upperBound) {
        this.lowerBound = lowerBound.clone();
        this.upperBound = upperBound.clone();
    }

    public AABB(Vec3 lowerBound) {
        this.lowerBound = lowerBound.clone();
        this.upperBound = new Vec3();
    }

    public Vec3 getLowerBound() {
        return lowerBound;
    }

    public void setLowerBound(Vec3 lowerBound) {
        this.lowerBound = lowerBound.clone();
    }

    public Vec3 getUpperBound() {
        return upperBound;
    }

    public void setUpperBound(Vec3 upperBound) {
        this.upperBound = upperBound.clone();
    }

    
    public void setFromPoints(List<Vec3> points) {
    	setFromPoints(points,new Vec3(), new Quaternion(), 0);
    }
    /**
   * Set the AABB bounds from a set of points.
   * @param points An array of Vec3's.
   * @return The self object
   */
    public void setFromPoints(List<Vec3> points, Vec3 position, Quaternion quaternion, double skinSize) {
        Vec3 l = lowerBound;
        Vec3 u = upperBound;

        // Set to the first point
        l.copy(points.get(0));
        if (quaternion != null) {
            quaternion.vmult(l, l);
        }
        u.copy(l);

        for (int i = 1; i < points.size(); i++) {
            Vec3 p = points.get(i);

            if (quaternion != null) {
                quaternion.vmult(p, tmp);
                p = tmp.clone();
            }

            if (p.x > u.x) u.x = p.x;
            if (p.x < l.x) l.x = p.x;
            if (p.y > u.y) u.y = p.y;
            if (p.y < l.y) l.y = p.y;
            if (p.z > u.z) u.z = p.z;
            if (p.z < l.z) l.z = p.z;
        }

        // Add offset
        if (position != null) {
            position.vadd(l, l);
            position.vadd(u, u);
        }

        if (skinSize != 0) {
            l.x -= skinSize;
            l.y -= skinSize;
            l.z -= skinSize;
            u.x += skinSize;
            u.y += skinSize;
            u.z += skinSize;
        }
    }

   
    public AABB copy() {
        return new AABB(lowerBound.clone(), upperBound.clone());
    }
     /**
   * Copy bounds from an AABB to this AABB
   * @param aabb Source to copy from
   * @return The this object, for chainability
   */
    public AABB copy(AABB aabb) {
        lowerBound.copy(aabb.lowerBound);
        upperBound.copy(aabb.upperBound);
        return this;
    }
    /**
   * Clone an AABB
   */
    public AABB clone(){
        return new AABB().copy(this);
    }

    /**
   * Extend this AABB so that it covers the given AABB too.
   */
    public void extend(AABB aabb) {
        lowerBound.x = Math.min(lowerBound.x, aabb.lowerBound.x);
        upperBound.x = Math.max(upperBound.x, aabb.upperBound.x);
        lowerBound.y = Math.min(lowerBound.y, aabb.lowerBound.y);
        upperBound.y = Math.max(upperBound.y, aabb.upperBound.y);
        lowerBound.z = Math.min(lowerBound.z, aabb.lowerBound.z);
        upperBound.z = Math.max(upperBound.z, aabb.upperBound.z);
    }

    public boolean overlaps(AABB aabb) {
        Vec3 l1 = lowerBound;
        Vec3 u1 = upperBound;
        Vec3 l2 = aabb.lowerBound;
        Vec3 u2 = aabb.upperBound;

        //      l2        u2
        //      |---------|
        // |--------|
        // l1       u1
        boolean overlapsX = (l2.x <= u1.x && u1.x <= u2.x) || (l1.x <= u2.x && u2.x <= u1.x);
        boolean overlapsY = (l2.y <= u1.y && u1.y <= u2.y) || (l1.y <= u2.y && u2.y <= u1.y);
        boolean overlapsZ = (l2.z <= u1.z && u1.z <= u2.z) || (l1.z <= u2.z && u2.z <= u1.z);

        return overlapsX && overlapsY && overlapsZ;
    }

    // Mostly for debugging
    public double volume() {
        Vec3 l = lowerBound;
        Vec3 u = upperBound;
        return (u.x - l.x) * (u.y - l.y) * (u.z - l.z);
    }

    /**
   * Returns true if the given AABB is fully contained in this AABB.
   */
    public boolean contains(AABB aabb) {
        Vec3 l1 = lowerBound;
        Vec3 u1 = upperBound;
        Vec3 l2 = aabb.lowerBound;
        Vec3 u2 = aabb.upperBound;

        // .....l2        u2
        // .....|---------|
        // |---------------|
        // l1              u1

        return l1.x <= l2.x && u1.x >= u2.x &&
               l1.y <= l2.y && u1.y >= u2.y &&
               l1.z <= l2.z && u1.z >= u2.z;
    }

    public void getCorners(Vec3 a, Vec3 b, Vec3 c, Vec3 d, Vec3 e, Vec3 f, Vec3 g, Vec3 h) {
        Vec3 l = lowerBound;
        Vec3 u = upperBound;

        a.copy(l);
        b.set(u.x, l.y, l.z);
        c.set(u.x, u.y, l.z);
        d.set(l.x, u.y, u.z);
        e.set(u.x, l.y, u.z);
        f.set(l.x, u.y, l.z);
        g.set(l.x, l.y, u.z);
        h.copy(u);
    }

    /**
   * Get the representation of an AABB in another frame.
   * @return The "target" AABB object.
   */
    public AABB toLocalFrame(Transform frame, AABB target) {
        // Get corners in current frame
        Vec3[] corners = transformIntoFrameCorners();

        // Transform them to new local frame
        for (int i = 0; i < 8; i++) {
            frame.pointToLocal(corners[i], corners[i]);
        }
        target.setFromPoints(Arrays.asList(corners));
        return target;
    }

    /**
   * Get the representation of an AABB in the global frame.
   * @return The "target" AABB object.
   */
    public AABB toWorldFrame(Transform frame, AABB target) {
        // Get corners in current frame
        Vec3[] corners = transformIntoFrameCorners();

        // Transform them to new local frame
        for (int i = 0; i < 8; i++) {
            frame.pointToWorld(corners[i], corners[i]);
        }
        target.setFromPoints(Arrays.asList(corners));
        return target;
    }

    /**
   * Check if the AABB is hit by a ray.
   */
    public boolean overlapsRay(Ray ray) {
        Vec3 direction = ray.direction;
        Vec3 from = ray.from;

    // ray.direction is unit direction vector of ray
        double dirFracX = 1.0 / direction.x;
        double dirFracY = 1.0 / direction.y;
        double dirFracZ = 1.0 / direction.z;

        // this.lowerBound is the corner of AABB with minimal coordinates - left bottom, rt is maximal corner
        double t1 = (lowerBound.x - from.x) * dirFracX;
        double t2 = (upperBound.x - from.x) * dirFracX;
        double t3 = (lowerBound.y - from.y) * dirFracY;
        double t4 = (upperBound.y - from.y) * dirFracY;
        double t5 = (lowerBound.z - from.z) * dirFracZ;
        double t6 = (upperBound.z - from.z) * dirFracZ;

        double tmin = Math.max(Math.max(Math.min(t1, t2), Math.min(t3, t4)), Math.min(t5, t6));
        double tmax = Math.min(Math.min(Math.max(t1, t2), Math.max(t3, t4)), Math.max(t5, t6));

    // if tmax < 0, ray (line) is intersecting AABB, but whole AABB is behing us
        if (tmax < 0) {
            return false;
        }
        // if tmin > tmax, ray doesn't intersect AABB
        if (tmin > tmax) {
            return false;
        }

        return true;
    }

    private Vec3[] transformIntoFrameCorners() {
        Vec3[] corners = new Vec3[8];
        for (int i = 0; i < 8; i++) {
            corners[i] = new Vec3();
        }

        Vec3 l = lowerBound;
        Vec3 u = upperBound;

        corners[0].copy(l);
        corners[1].set(u.x, l.y, l.z);
        corners[2].set(u.x, u.y, l.z);
        corners[3].set(l.x, u.y, u.z);
        corners[4].set(u.x, l.y, u.z);
        corners[5].set(l.x, u.y, l.z);
        corners[6].set(l.x, l.y, u.z);
        corners[7].copy(u);

        return corners;
    }

    // Rest of the code as-is...
}
