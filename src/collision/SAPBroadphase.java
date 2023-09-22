package collision;

import java.util.ArrayList;
import java.util.EventListener;
import java.util.List;

import math.Vec3;
import math.JacobianElement;
import math.Mat3; 
import objects.Body;
import shapes.Shape;

import world.World;

/**
 * Sweep and prune broadphase along one axis.
 */
public class SAPBroadphase extends Broadphase {

    /**
     * List of bodies currently in the broadphase.
     */
    private List<Body> axisList;

    /**
     * The world to search in.
     */
    private World world;

    /**
     * Axis to sort the bodies along.
     * Set to 0 for x axis, and 1 for y axis.
     * For best performance, pick the axis where bodies are most distributed.
     */
    private int axisIndex;

    private final EventListener _addBodyHandler;
    private final EventListener _removeBodyHandler;

    /**
     * Constructor for SAPBroadphase.
     * @param world The world to use for the broadphase.
     */
    public SAPBroadphase(World world) {
        super();

        this.axisList = new ArrayList<>();
        this.world = null;
        this.axisIndex = 0;

        this._addBodyHandler = (event) -> {
            axisList.add(event.body);
        };

        this._removeBodyHandler = (event) -> {
            int idx = axisList.indexOf(event.body);
            if (idx != -1) {
                axisList.remove(idx);
            }
        };

        if (world != null) {
            setWorld(world);
        }
    }

    /**
     * Change the world.
     * @param world The new physics world.
     */
    public void setWorld(World world) {
        // Clear the old axis array
        this.axisList.clear();

        // Add all bodies from the new world
        for (int i = 0; i < world.bodies.size(); i++) {
            this.axisList.add(world.bodies.get(i));
        }

        // Remove old handlers, if any
        world.removeEventListener('addBody', this._addBodyHandler);
        world.removeEventListener('removeBody', this._removeBodyHandler);

        // Add handlers to update the list of bodies
        world.addEventListener('addBody', this._addBodyHandler);
        world.addEventListener('removeBody', this._removeBodyHandler);

        this.world = world;
        this.dirty = true;
    }

    /**
     * Collect all collision pairs.
     * @param world The physics world.
     * @param pairs1 An array to store the first bodies in collision pairs.
     * @param pairs2 An array to store the second bodies in collision pairs.
     */
    @Override
    public void collisionPairs(World world, List<Body> pairs1, List<Body> pairs2) {
        List<Body> bodies = this.axisList;
        int N = bodies.size();
        int axisIndex = this.axisIndex;

        if (this.dirty) {
            this.sortList();
            this.dirty = false;
        }

        // Look through the list
        for (int i = 0; i < N; i++) {
            Body bi = bodies.get(i);

            for (int j = i + 1; j < N; j++) {
                Body bj = bodies.get(j);

                if (!this.needBroadphaseCollision(bi, bj)) {
                    continue;
                }

                if (!SAPBroadphase.checkBounds(bi, bj, axisIndex)) {
                    break;
                }

                this.intersectionTest(bi, bj, pairs1, pairs2);
            }
        }
    }

    /**
     * Sort the list of bodies along the selected axis.
     */
    private void sortList() {
        List<Body> axisList = this.axisList;
        int axisIndex = this.axisIndex;
        int N = axisList.size();

        // Update AABBs
        for (int i = 0; i < N; i++) {
            Body bi = axisList.get(i);
            if (bi.aabbNeedsUpdate) {
                bi.updateAABB();
            }
        }

        // Sort the list
        if (axisIndex == 0) {
            SAPBroadphase.insertionSortX(axisList);
        } else if (axisIndex == 1) {
            SAPBroadphase.insertionSortY(axisList);
        } else if (axisIndex == 2) {
            SAPBroadphase.insertionSortZ(axisList);
        }
    }

    /**
     * Computes the variance of the body positions and estimates the best axis to use.
     * Will automatically set the property `axisIndex`.
     */
    public void autoDetectAxis() {
        double sumX = 0;
        double sumX2 = 0;
        double sumY = 0;
        double sumY2 = 0;
        double sumZ = 0;
        double sumZ2 = 0;
        List<Body> bodies = this.axisList;
        int N = bodies.size();
        double invN = 1.0 / N;

        for (int i = 0; i < N; i++) {
            Body b = bodies.get(i);

            double centerX = b.position.x;
            sumX += centerX;
            sumX2 += centerX * centerX;

            double centerY = b.position.y;
            sumY += centerY;
            sumY2 += centerY * centerY;

            double centerZ = b.position.z;
            sumZ += centerZ;
            sumZ2 += centerZ * centerZ;
        }

        double varianceX = sumX2 - sumX * sumX * invN;
        double varianceY = sumY2 - sumY * sumY * invN;
        double varianceZ = sumZ2 - sumZ * sumZ * invN;

        if (varianceX > varianceY) {
            if (varianceX > varianceZ) {
                this.axisIndex = 0;
            } else {
                this.axisIndex = 2;
            }
        } else if (varianceY > varianceZ) {
            this.axisIndex = 1;
        } else {
            this.axisIndex = 2;
        }
    }

    /**
     * Returns all the bodies within an AABB.
     * @param world The physics world.
     * @param aabb The AABB to query.
     * @param result An array to store resulting bodies in.
     * @return A list of bodies that are within the specified AABB.
     */
    public List<Body> aabbQuery(World world, AABB aabb, List<Body> result) {
        if (this.dirty) {
            this.sortList();
            this.dirty = false;
        }
        /*
         * @UNUSED CODE
        int axisIndex = this.axisIndex;
        String axis = "x";
        if (axisIndex == 1) {
            axis = "y";
        }
        if (axisIndex == 2) {
            axis = "z";
        }
        double lower = aabb.lowerBound[axis];
        double upper = aabb.upperBound[axis];
         */
        List<Body> axisList = this.axisList;

        for (int i = 0; i < axisList.size(); i++) {
            Body b = axisList.get(i);

            if (b.aabbNeedsUpdate) {
                b.updateAABB();
            }

            if (b.aabb.overlaps(aabb)) {
                result.add(b);
            }
        }

        return result;
    }

    // Static helper methods

    /**
     * Check if the bounds of two bodies overlap, along the given SAP axis.
     * @param bi The first body.
     * @param bj The second body.
     * @param axisIndex The axis index to check (0 for x, 1 for y, 2 for z).
     * @return True if the bounds overlap, false otherwise.
     */
    public static boolean checkBounds(Body bi, Body bj, int axisIndex) {
        double biPos, bjPos;

        if (axisIndex == 0) {
            biPos = bi.position.x;
            bjPos = bj.position.x;
        } else if (axisIndex == 1) {
            biPos = bi.position.y;
            bjPos = bj.position.y;
        } else if (axisIndex == 2) {
            biPos = bi.position.z;
            bjPos = bj.position.z;
        }

        double ri = bi.boundingRadius;
        double rj = bj.boundingRadius;
        double boundA2 = biPos + ri;
        double boundB1 = bjPos - rj;

        return boundB1 < boundA2;
    }

    /**
     * Perform an insertion sort on an array of bodies along the X axis.
     * @param a The array of bodies to sort.
     * @return The sorted array.
     */
    public static List<Body> insertionSortX(List<Body> a) {
        int l = a.size();
        for (int i = 1; i < l; i++) {
            Body v = a.get(i);
            int j;
            for (j = i - 1; j >= 0; j--) {
                if (a.get(j).aabb.lowerBound.x <= v.aabb.lowerBound.x) {
                    break;
                }
                a.set(j + 1, a.get(j));
            }
            a.set(j + 1, v);
        }
        return a;
    }

    /**
     * Perform an insertion sort on an array of bodies along the Y axis.
     * @param a The array of bodies to sort.
     * @return The sorted array.
     */
    public static List<Body> insertionSortY(List<Body> a) {
        int l = a.size();
        for (int i = 1; i < l; i++) {
            Body v = a.get(i);
            int j;
            for (j = i - 1; j >= 0; j--) {
                if (a.get(j).aabb.lowerBound.y <= v.aabb.lowerBound.y) {
                    break;
                }
                a.set(j + 1, a.get(j));
            }
            a.set(j + 1, v);
        }
        return a;
    }

    /**
     * Perform an insertion sort on an array of bodies along the Z axis.
     * @param a The array of bodies to sort.
     * @return The sorted array.
     */
    public static List<Body> insertionSortZ(List<Body> a) {
        int l = a.size();
        for (int i = 1; i < l; i++) {
            Body v = a.get(i);
            int j;
            for (j = i - 1; j >= 0; j--) {
                if (a.get(j).aabb.lowerBound.z <= v.aabb.lowerBound.z) {
                    break;
                }
                a.set(j + 1, a.get(j));
            }
            a.set(j + 1, v);
        }
        return a;
    }

}
