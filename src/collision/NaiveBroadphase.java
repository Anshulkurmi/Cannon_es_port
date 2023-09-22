package collision;

import java.util.List;

import objects.Body;
import world.World;

/**
 * Naive broadphase implementation, used in lack of better ones.
 *
 * The naive broadphase looks at all possible pairs without restriction, therefore it has complexity N^2 _(which is bad)_
 */
public class NaiveBroadphase extends Broadphase {

    /**
     * Constructor for the NaiveBroadphase.
     * @todo Remove useless constructor
     */
    public NaiveBroadphase() {
        super();
    }

    /**
     * Get all the collision pairs in the physics world.
     * @param world The physics world.
     * @param pairs1 An array to store the first bodies in collision pairs.
     * @param pairs2 An array to store the second bodies in collision pairs.
     */
    @Override
    public void collisionPairs(World world, List<Body> pairs1, List<Body> pairs2) {
        List<Body> bodies = world.bodies;
        int n = bodies.size();
        Body bi, bj;

        // Naive N^2 collision detection
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < i; j++) {
                bi = bodies.get(i);
                bj = bodies.get(j);

                if (!this.needBroadphaseCollision(bi, bj)) {
                    continue;
                }

                this.intersectionTest(bi, bj, pairs1, pairs2);
            }
        }
    }

    /**
     * Returns all the bodies within an AABB (Axis-Aligned Bounding Box).
     * @param world The physics world.
     * @param aabb The AABB to query.
     * @param result An array to store resulting bodies in.
     * @return A list of bodies that are within the specified AABB.
     */
    public List<Body> aabbQuery(World world, AABB aabb, List<Body> result) {
        List<Body> bodies = world.bodies;

        for (int i = 0; i < bodies.size(); i++) {
            Body b = bodies.get(i);

            if (b.aabbNeedsUpdate) {
                b.updateAABB();
            }

            // Check if the body's AABB overlaps with the given AABB
            if (b.aabb.overlaps(aabb)) {
                result.add(b);
            }
        }

        return result;
    }
}
