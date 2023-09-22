package utils;

import collision.AABB;

/**
 * Octree
 */
public class Octree extends OctreeNode {
    public int maxDepth; // Maximum subdivision depth

    //added default constructor
    public Octree() {
        this(new AABB() , 8);
    }

    public Octree(AABB aabb, int maxDepth) {
        super();
        this.maxDepth = maxDepth;
    }

}
