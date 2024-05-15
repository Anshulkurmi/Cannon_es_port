package collision;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import world.World;
import objects.Body;
import math.Quaternion;
import math.Vec3;
import shapes.Shape;

/**
 * Base class for broadphase implementations
 * 
 * @author schteppe
 */

public class Broadphase {
    private World world; // The world to search for collisions in.
    private boolean useBoundingBoxes; // If set to true, the broadphase uses bounding boxes for intersection tests,
                                      // else it uses bounding spheres.
    public boolean dirty; // Set to true if the objects in the world moved.

    public Broadphase() {
        this.world = null;
        this.useBoundingBoxes = false;
        this.dirty = true;
    }

    /**
     * Get the collision pairs from the world
     *
     * @param world The world to search in
     * @param p1    Empty array to be filled with body objects
     * @param p2    Empty array to be filled with body objects
     */
    public void collisionPairs(World world, List<Body> p1, List<Body> p2) {
        throw new UnsupportedOperationException("collisionPairs not implemented for this BroadPhase class!");
    }

    /**
     * Check if a body pair needs to be intersection tested at all.
     */
    public boolean needBroadphaseCollision(Body bodyA, Body bodyB) {
        // Check collision filter masks
        if ((bodyA.collisionFilterGroup & bodyB.collisionFilterMask) == 0
                || (bodyB.collisionFilterGroup & bodyA.collisionFilterMask) == 0) {
            return false;
        }

        // Check types
        if (((bodyA.type & Body.STATIC) != 0 || bodyA.sleepState == Body.SLEEPING)
                && ((bodyB.type & Body.STATIC) != 0 || bodyB.sleepState == Body.SLEEPING)) {
            // Both bodies are static or sleeping. Skip.
            return false;
        }

        return true;
    }

    /**
     * Check if the bounding volumes of two bodies intersect.
     */
    public void intersectionTest(Body bodyA, Body bodyB, List<Body> pairs1, List<Body> pairs2) {
        if (this.useBoundingBoxes) {
            this.doBoundingBoxBroadphase(bodyA, bodyB, pairs1, pairs2);
        } else {
            this.doBoundingSphereBroadphase(bodyA, bodyB, pairs1, pairs2);
        }
    }

    /**
     * Check if the bounding spheres of two bodies are intersecting.
     *
     * @param pairs1 bodyA is appended to this array if intersection
     * @param pairs2 bodyB is appended to this array if intersection
     */
    public void doBoundingSphereBroadphase(Body bodyA, Body bodyB, List<Body> pairs1, List<Body> pairs2) {
        Vec3 r = Broadphase_collisionPairs_r;
        bodyB.position.vsub(bodyA.position, r);
        double boundingRadiusSum2 = Math.pow(bodyA.boundingRadius + bodyB.boundingRadius, 2);
        double norm2 = r.lengthSquared();
        if (norm2 < boundingRadiusSum2) {
            pairs1.add(bodyA);
            pairs2.add(bodyB);
        }
    }

    /**
     * Check if the bounding boxes of two bodies are intersecting.
     */
    public void doBoundingBoxBroadphase(Body bodyA, Body bodyB, List<Body> pairs1, List<Body> pairs2) {
        if (bodyA.aabbNeedsUpdate) {
            bodyA.updateAABB();
        }
        if (bodyB.aabbNeedsUpdate) {
            bodyB.updateAABB();
        }

        // Check AABB / AABB
        if (bodyA.aabb.overlaps(bodyB.aabb)) {
            pairs1.add(bodyA);
            pairs2.add(bodyB);
        }
    }

    /**
     * Removes duplicate pairs from the pair arrays.
     */
    public void makePairsUnique(List<Body> pairs1, List<Body> pairs2) {
        Map<String, Integer> temp =  Broadphase_makePairsUnique_temp;
        List<Body> p1 = Broadphase_makePairsUnique_p1 ; //new ArrayList<>(pairs1);
        List<Body> p2 = Broadphase_makePairsUnique_p2 ; //new ArrayList<>(pairs2);
        int N = p1.size();

        //added this 
        for(int i = 0; i < N; i++) {
            p1.add(pairs1.get(i));
            p2.add(pairs1.get(i));     
        }

        pairs1.clear();
        pairs2.clear();

        for (int i = 0; i < N; i++) {
            int id1 = p1.get(i).id;
            int id2 = p2.get(i).id;
            String key = id1 < id2 ? id1 + "-" + id2 : id2 + "-" + id1;
            temp.put(key, i);
            temp.keySet().add(key); // added this : wasn't there already
        }

        for (String key : temp.keySet()) {
            int pairIndex = temp.get(key);
            pairs1.add(p1.get(pairIndex));
            pairs2.add(p2.get(pairIndex));
            temp.remove(key);//added this : was missing initially 
        }
    }

    /**
     * To be implemented by subclasses
     */
    public void setWorld(World world) {
    }

    /**
     * Check if the bounding spheres of two bodies overlap.
     */
    public static boolean boundingSphereCheck(Body bodyA, Body bodyB) {
        Vec3 dist = new Vec3();
        bodyA.position.vsub(bodyB.position, dist);
        Shape sa = bodyA.shapes.get(0);
        Shape sb = bodyB.shapes.get(0);
        return Math.pow(sa.boundingSphereRadius + sb.boundingSphereRadius, 2) > dist.lengthSquared();
    }

    /**
     * Returns all the bodies within the AABB.
     */
    public List<Body> aabbQuery(World world, AABB aabb, List<Body> result) {
        System.out.println(".aabbQuery is not implemented in this Broadphase subclass.");
        return new ArrayList<>();
    }

    // Temp objects
    static Vec3 Broadphase_collisionPairs_r = new Vec3();

    static Vec3 Broadphase_collisionPairs_normal = new Vec3();
    static Quaternion Broadphase_collisionPairs_quat = new Quaternion();
    static Vec3 Broadphase_collisionPairs_relpos = new Vec3();

    Map<String,Integer>  Broadphase_makePairsUnique_temp  = new HashMap<>();
    List<Body> Broadphase_makePairsUnique_p1= new ArrayList<>();
    List<Body> Broadphase_makePairsUnique_p2= new ArrayList<>();
     
    static Vec3 bsc_dist=new Vec3();
}
