//check aabbQuery method
//this.root.maxDepth in insert method

package utils;

import java.util.ArrayList;
import java.util.List;

// Import the necessary classes from the same package
import collision.AABB;
import math.Vec3;
import math.Transform;
import collision.Ray;

/**
 * OctreeNode
 */
public class OctreeNode {
    public OctreeNode root; // The root node
    public AABB aabb; // Boundary of this node
    public List<Integer> data; // Contained data at the current node level
    public List<OctreeNode> children; // Children to this node

    public OctreeNode() {
        this( null , new AABB() , new ArrayList<>() ,new ArrayList<>());
    }

    //added constructor
    public OctreeNode(AABB aabb){
        this(null , aabb ,new ArrayList<>(),new ArrayList<>());
    }    

    /** The root node */
    // root?: Octree | null
    /** Boundary of this node */
    //aabb?: AABB

    //change OctreeNode to Octree in the constructor 
    public OctreeNode(Octree root ,AABB aabb,List<Integer> data , List<OctreeNode> children) {
    	this.root = root;
    	this.aabb = aabb ;
    	this.data = data;
        this.children = children;
    }

    /**
   * reset
   */
    public void reset() {
        this.children.clear();
        this.data.clear();
    }

    /**
   * Insert data into this node
   * @return True if successful, otherwise false
   */
    public boolean insert(AABB aabb, int elementData, int level) {
        // Ignore objects that do not belong in this node
        if (!this.aabb.contains(aabb)) {
            return false; // Object cannot be added
        }

        List<OctreeNode> children = this.children;
        int maxDepth = 0;
        if(this instanceof Octree)
             maxDepth =  ((Octree)this).maxDepth; // Maximum subdivision depth
        else if(this.root instanceof Octree)
              maxDepth =  ((Octree)this.root).maxDepth;
          
        if (level < maxDepth) {
            // Subdivide if there are no children yet
            boolean subdivided = false;
            if (children.isEmpty()) {
                this.subdivide();
                subdivided = true;
            }

            // Add to whichever node will accept it
            for (int i = 0; i < 8; i++) {
                if (children.get(i).insert(aabb, elementData, level + 1)) {
                    return true;
                }
            }

            if (subdivided) {
                // No children accepted! Might as well just remove them since they contain none
                children.clear();
            }
        }

        // Too deep, or children didn't want it. Add it to the current node
        this.data.add(elementData);

        return true;
    }

    /**
     * Create 8 equally sized children nodes and put them in the `children` array.
     */
    public void subdivide() {
    	AABB aabb = this.aabb ;
        Vec3 l = aabb.lowerBound;
        Vec3 u = aabb.upperBound;

        List<OctreeNode> children = this.children;

        children.add(new OctreeNode(new AABB(new Vec3(0, 0, 0))));
        children.add(new OctreeNode(new AABB(new Vec3(1, 0, 0))));
        children.add(new OctreeNode(new AABB(new Vec3(1, 1, 0))));
        children.add(new OctreeNode(new AABB(new Vec3(1, 1, 1))));
        children.add(new OctreeNode(new AABB(new Vec3(0, 1, 1))));
        children.add(new OctreeNode(new AABB(new Vec3(0, 0, 1))));
        children.add(new OctreeNode(new AABB(new Vec3(1, 0, 1))));
        children.add(new OctreeNode(new AABB(new Vec3(0, 1, 0))));

        //Vec3 halfDiagonal = new Vec3();
        u.vsub(l, halfDiagonal);
        halfDiagonal.scale(0.5, halfDiagonal);

        //changed Octree to OctreeNode
        OctreeNode root = (this.root != null) ? this.root : (Octree) this;

        for (int i = 0; i < 8; i++) {
            OctreeNode child = children.get(i);

            // Set current node as root
            child.root = root;

            // Compute bounds
            Vec3 lowerBound = child.aabb.lowerBound;
            lowerBound.x *= halfDiagonal.x;
            lowerBound.y *= halfDiagonal.y;
            lowerBound.z *= halfDiagonal.z;

            lowerBound.vadd(l, lowerBound);

            // Upper bound is always lower bound + halfDiagonal
            lowerBound.vadd(halfDiagonal, child.aabb.upperBound);
        }
    }

    /**
   * Get all data, potentially within an AABB
   * @return The "result" object
   */
    public List<Integer> aabbQuery(AABB aabb, List<Integer> result) {
        // Implement AABB query method
        // Add objects at this level
        result.addAll(data);

        List<OctreeNode> children = this.children;

        // Recursively query child nodes
        for (int i = 0; i < children.size(); i++) {
            OctreeNode child = children.get(i);
            if (child.aabb.overlaps(aabb)) {
                child.aabbQuery(aabb, result);
            }
        }

        return result;
    }

    /**
   * Get all data, potentially intersected by a ray.
   * @return The "result" object
   */
    public List<Integer> rayQuery(Ray ray, Transform treeTransform, List<Integer> result) {
        // Use AABB query for now, implement real ray query which needs fewer lookups
        /** @todo implement real ray query which needs less lookups */

        ray.getAABB(tmpAABB);
        tmpAABB.toLocalFrame(treeTransform, tmpAABB);
        this.aabbQuery(tmpAABB, result);

        return result;
    }

    /**
   * removeEmptyNodes
   */
    public void removeEmptyNodes() {
        for (int i = this.children.size() - 1; i >= 0; i--) {
            OctreeNode child = this.children.get(i);
            child.removeEmptyNodes();
            if (child.children.isEmpty() && child.data.isEmpty()) {
                children.remove(i);
            }
        }
    }

    static AABB tmpAABB = new AABB();
    static Vec3 halfDiagonal = new Vec3();
}


