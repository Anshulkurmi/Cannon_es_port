package constraints;

import math.Vec3;

public class ConeTwistConstraintOptions {
    // Define fields that correspond to the constructor parameters of ConeTwistConstraint
    // For example:
    protected Vec3 pivotA;
    protected Vec3 pivotB;
    protected Vec3 axisA;//quaternion to vec3
    protected Vec3 axisB;
    protected double maxForce;
    protected double maxBias;
    protected double relaxation;
	public boolean collideConnected;
	public double angle;
	public double twistAngle ; // added twistAngle 
    
    public ConeTwistConstraintOptions() {
    	this(new Vec3(),new Vec3(),new Vec3(),new Vec3(),1e6 , 0,0 , false , 0 , 0);
    }
    // Create a constructor to initialize the fields
    public ConeTwistConstraintOptions(Vec3 pivotA, Vec3 pivotB, Vec3 axisA, Vec3 axisB, double maxForce, double maxBias, double relaxation, boolean collideConnected, double angle,double twistAngle) {
        this.pivotA = pivotA;
        this.pivotB = pivotB;
        this.axisA = axisA;
        this.axisB = axisB;
        this.maxForce = maxForce;
        this.maxBias = maxBias;
        this.relaxation = relaxation;
        this.collideConnected = collideConnected;
        this.angle = angle ;
        this.twistAngle = twistAngle ;
    }

 
}
