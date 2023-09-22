package constraints;

import objects.Body;

public class LockConstraintOptions {
    // Define fields that correspond to the constructor parameters of LockConstraint
    // For example:
    protected Body bodyA;
    protected Body bodyB;
    protected double maxForce;
    
    public LockConstraintOptions(Body bodyA, Body bodyB) {
    	this(bodyA,bodyB,1e6);
    }

    // Create a constructor to initialize the fields
    public LockConstraintOptions(Body bodyA, Body bodyB, double maxForce) {
        this.bodyA = bodyA;
        this.bodyB = bodyB;
        this.maxForce = maxForce;
    }

    // Getters and setters for the fields (if needed)
    // ...
}
