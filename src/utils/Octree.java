package utils;

import collision.AABB;

/**
 * Octree
 */
public class Octree extends OctreeNode {
	/**
	 * Maximum subdivision depth
	 * @default 8
	 */
    public int maxDepth; // 

    //added default constructor
    public Octree() {
        this(new AABB() , 8);
    }

    public Octree(AABB aabb, int maxDepth) {
        super(aabb);
        this.maxDepth = maxDepth;
    }

}
