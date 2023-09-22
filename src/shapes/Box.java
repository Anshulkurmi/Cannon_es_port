package shapes;



import math.Vec3; 
import java.util.ArrayList;
import java.util.List;

import math.Quaternion; 
 
/**
 * A 3d box shape.
 * @example
 *     const size = 1
 *     const halfExtents = new CANNON.Vec3(size, size, size)
 *     const boxShape = new CANNON.Box(halfExtents)
 *     const boxBody = new CANNON.Body({ mass: 1, shape: boxShape })
 *     world.addBody(boxBody)
 */
public class Box extends Shape {
    /**
     * The half extents of the box.
     */
    private Vec3 halfExtents;

    /**
     * Used by the contact generator to make contacts with other convex polyhedra for example.
     */
    public ConvexPolyhedron convexPolyhedronRepresentation;

    
    public Box(Vec3 halfExtents) {
    	this(halfExtents,null);
    }
    	   
    /**
     * Constructs a box shape with the given half extents.
     *
     * @param halfExtents The half extents of the box.
     */
    public Box(Vec3 halfExtents,ShapeOptions options) {
        super(ShapeTypes.BOX,options);

        this.halfExtents = halfExtents;
        this.convexPolyhedronRepresentation = null;
        updateConvexPolyhedronRepresentation();
        updateBoundingSphereRadius();
    }

    /**
     * Updates the local convex polyhedron representation used for some collisions.
     */
    private void updateConvexPolyhedronRepresentation() {
        double sx = halfExtents.x;
        double sy = halfExtents.y;
        double sz = halfExtents.z;

        // Vec3[] vertices = {
        //     new Vec3(-sx, -sy, -sz),
        //     new Vec3(sx, -sy, -sz),
        //     new Vec3(sx, sy, -sz),
        //     new Vec3(-sx, sy, -sz),
        //     new Vec3(-sx, -sy, sz),
        //     new Vec3(sx, -sy, sz),
        //     new Vec3(sx, sy, sz),
        //     new Vec3(-sx, sy, sz)
        // };

        List<Vec3> vertices =  new ArrayList<Vec3>();
        vertices.add( new Vec3(-sx, -sy, -sz));
        vertices.add( new Vec3(sx, -sy, -sz)) ;
        vertices.add(new Vec3(sx, sy, -sz));
        vertices.add(new Vec3(-sx, sy, -sz));
        vertices.add(new Vec3(-sx, -sy, sz));
        vertices.add(new Vec3(sx, -sy, sz));
        vertices.add(new Vec3(sx, sy, sz));
        vertices.add(new Vec3(-sx, sy, sz));

        // int[][] faces = {
        //     {3, 2, 1, 0}, // -z
        //     {4, 5, 6, 7}, // +z
        //     {5, 4, 0, 1}, // -y
        //     {2, 3, 7, 6}, // +y
        //     {0, 4, 7, 3}, // -x
        //     {1, 2, 6, 5}  // +x
        // };

        List<int[]> faces = new ArrayList<>();
        faces.add(new int[] {3, 2, 1, 0});
        faces.add(new int[] {4, 5, 6, 7});
        faces.add(new int[] {5, 4, 0, 1});
        faces.add(new int[] {2, 3, 7, 6});
        faces.add(new int[] {0, 4, 7, 3});
        faces.add(new int[] {1, 2, 6, 5});

        // Vec3[] axes = {
        //     new Vec3(0, 0, 1),
        //     new Vec3(0, 1, 0),
        //     new Vec3(1, 0, 0)
        // };
        
        List<Vec3> axes = new ArrayList<Vec3>();
        axes.add(new Vec3(0, 0, 1));
        axes.add(new Vec3(0, 1, 0));
         axes.add(new Vec3(0, 0, 1));

        //changed vertices , axes , faces types  to lists from arrays and updated convexPolyhedron h 
        ConvexPolyhedron h = new ConvexPolyhedron(vertices, faces,new ArrayList<>(), axes,0.0);
        h.material= this.material;
        convexPolyhedronRepresentation = h;
    }

    /**
     * Calculates the inertia of the box.
     *
     * @param mass   The mass of the box.
     * @param target The target vector to store the inertia.
     * @return The target vector with calculated inertia.
     */
    @Override
    public Vec3 calculateLocalInertia(double mass, Vec3 target) {
        Box.calculateInertia(halfExtents, mass, target);
        return target;
    }

    /**
     * Static method to calculate inertia for a box shape.
     *
     * @param halfExtents The half extents of the box.
     * @param mass        The mass of the box.
     * @param target      The target vector to store the inertia.
     */
    public static void calculateInertia(Vec3 halfExtents, double mass, Vec3 target) {
        double eX = halfExtents.x;
        double eY = halfExtents.y;
        double eZ = halfExtents.z;

        target.x = (1.0 / 12.0) * mass * (2 * eY * 2 * eY + 2 * eZ * 2 * eZ);
        target.y = (1.0 / 12.0) * mass * (2 * eX * 2 * eX + 2 * eZ * 2 * eZ);
        target.z = (1.0 / 12.0) * mass * (2 * eY * 2 * eY + 2 * eX * 2 * eX);
    }

    /**
     * Get the box 6 side normals.
     *
     * @param sixTargetVectors An array of 6 vectors to store the resulting side normals.
     * @param quat             Orientation to apply to the normal vectors. If not provided, the vectors will be in respect to the local frame.
     * @return An array of 6 vectors representing the side normals.
     */
    public Vec3[] getSideNormals(Vec3[] sixTargetVectors, Quaternion quat) {
        Vec3[] sides = sixTargetVectors;
        double exX = halfExtents.x;
        double exY = halfExtents.y;
        double exZ = halfExtents.z;

        sides[0].set(exX, 0, 0);
        sides[1].set(0, exY, 0);
        sides[2].set(0, 0, exZ);
        sides[3].set(-exX, 0, 0);
        sides[4].set(0, -exY, 0);
        sides[5].set(0, 0, -exZ);

        if (quat != null) {
            for (int i = 0; i < sides.length; i++) {
                quat.vmult(sides[i], sides[i]);
            }
        }

        return sides;
    }

    /**
     * Returns the volume of the box.
     *
     * @return The volume of the box.
     */
    @Override
    public double volume() {
        return 8.0 * halfExtents.x * halfExtents.y * halfExtents.z;
    }

    /**
     * Update the bounding sphere radius.
     */
    @Override
    public void updateBoundingSphereRadius() {
        boundingSphereRadius = halfExtents.length();
    }

    /**
     * For each world corner of the box, execute a callback function.
     *
     * @param pos      The position of the box in world coordinates.
     * @param quat     The rotation quaternion of the box.
     * @param callback The callback function to execute for each world corner.
     */
    public void forEachWorldCorner(Vec3 pos, Quaternion quat, BoxCornerCallback callback) {
        double eX = halfExtents.x;
        double eY = halfExtents.y;
        double eZ = halfExtents.z;

        double[][] corners = {
            {eX, eY, eZ},
            {-eX, eY, eZ},
            {-eX, -eY, eZ},
            {-eX, -eY, -eZ},
            {eX, -eY, -eZ},
            {eX, eY, -eZ},
            {-eX, eY, -eZ},
            {eX, -eY, eZ}
        };

        for (double[] corner : corners) {
            Vec3 worldCornerTempPos = new Vec3(corner[0], corner[1], corner[2]);
            quat.vmult(worldCornerTempPos, worldCornerTempPos);
            pos.vadd(worldCornerTempPos, worldCornerTempPos);
            callback.corner(worldCornerTempPos.x, worldCornerTempPos.y, worldCornerTempPos.z);
        }
    }

    /**
     * Calculate the world axis-aligned bounding box (AABB) of the box.
     *
     * @param pos The position of the box in world coordinates.
     * @param quat The rotation quaternion of the box.
     * @param min The minimum corner of the AABB to be calculated.
     * @param max The maximum corner of the AABB to be calculated.
     */
    @Override
    public void calculateWorldAABB(Vec3 pos, Quaternion quat, Vec3 min, Vec3 max) {
        double eX = halfExtents.x;
        double eY = halfExtents.y;
        double eZ = halfExtents.z;

        Vec3[] worldCornersTemp = new Vec3[]{
            new Vec3(eX, eY, eZ),
            new Vec3(-eX, eY, eZ),
            new Vec3(-eX, -eY, eZ),
            new Vec3(-eX, -eY, -eZ),
            new Vec3(eX, -eY, -eZ),
            new Vec3(eX, eY, -eZ),
            new Vec3(-eX, eY, -eZ),
            new Vec3(eX, -eY, eZ)
        };

        Vec3 wc = worldCornersTemp[0];
        quat.vmult(wc, wc);
        pos.vadd(wc, wc);
        max.copy(wc);
        min.copy(wc);

        for (int i = 1; i < 8; i++) {
            Vec3 currentCorner = worldCornersTemp[i];
            quat.vmult(currentCorner, currentCorner);
            pos.vadd(currentCorner, currentCorner);

            double x = currentCorner.x;
            double y = currentCorner.y;
            double z = currentCorner.z;

            if (x > max.x) {
                max.x = x;
            }
            if (y > max.y) {
                max.y = y;
            }
            if (z > max.z) {
                max.z = z;
            }

            if (x < min.x) {
                min.x = x;
            }
            if (y < min.y) {
                min.y = y;
            }
            if (z < min.z) {
                min.z = z;
            }
        }
    }

    /**
     * Interface for the corner callback function.
     */
    public interface BoxCornerCallback {
        void corner(double x, double y, double z);
    }
}
