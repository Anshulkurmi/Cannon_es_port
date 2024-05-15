package constraints;

import objects.Body;
import math.Vec3; ;


public class HingeConstraintOptions {
    // Define fields that correspond to the constructor parameters of HingeConstraint
    // For example:
    protected Body bodyA;
    protected Body bodyB;
    protected Vec3 pivotA;
    protected Vec3 pivotB;
    protected Vec3 axisA;
    protected Vec3 axisB;
    protected double maxForce;
    protected double maxTorque;
	private boolean collideConnected;
    
    
    public HingeConstraintOptions(Body bodyA,Body bodyB) {
    	this(bodyA,bodyB,new Vec3(),new Vec3(), new Vec3(),new Vec3(), 1e6 ,1e6);
    }
    
//    public HingeConstraintOptions(Vec3 position , Vec3 axis,boolean collideConnected) {
//    	this(bodyA, bodyB,)
//    }
//    		
    // Create a constructor to initialize the fields
    /**
     * @param bodyA
     * @param bodyB
     * @param pivotA
     * @param pivotB
     * @param axisA
     * @param axisB
     * @param maxForce 
     * @param maxTorque
     * 
     */
    
//    public HingeConstraintOptions(Vec3 pivotA , ) {
//    	
//    }
    public HingeConstraintOptions(Body bodyA, Body bodyB, Vec3 pivotA, Vec3 pivotB, Vec3 axisA, Vec3 axisB, double maxForce, double maxTorque) {
        this.bodyA = bodyA;
        this.bodyB = bodyB;
        this.pivotA = pivotA;
        this.pivotB = pivotB;
        this.axisA = axisA;
        this.axisB = axisB;
        this.maxForce = maxForce;
        this.maxTorque = maxTorque;
        this.collideConnected = false ;
    }

	public HingeConstraintOptions(Vec3 position, Vec3 axis, Vec3 zERO, Vec3 axis2, boolean b) {
		// TODO Auto-generated constructor stub
		this.pivotA = position ;
		this.axisA = axis ;
		this.pivotB = Vec3.ZERO ;
		this.axisB = axis ;
		this.collideConnected = false ;
		
	}

    // Getters and setters for the fields (if needed)
    // ...
}
