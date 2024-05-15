package shapes;

import material.Material;

public class ShapeOptions {
    // Define properties similar to the TypeScript code
    public boolean collisionResponse;
    public int collisionFilterGroup;
    public int collisionFilterMask;
    public Material material;
    
    public ShapeOptions() {
    	this.collisionResponse = true ;
    	this.collisionFilterGroup = 1;
    	this.collisionFilterMask = -1;
    	this.material = null ;
    }
    
    public ShapeOptions(boolean collisionResponse,int collisionFilterGroup , int collisionFilterMask ,Material material) {
    	this.collisionResponse = collisionResponse ;
    	this.collisionFilterGroup = collisionFilterGroup ;
    	this.collisionFilterMask = collisionFilterMask ;
    	this.material = material ;
    }
}