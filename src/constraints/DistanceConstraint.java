package constraints;

import equations.ContactEquation;
import objects.Body;
import math.Vec3;

/**
 * Constrains two bodies to be at a constant distance from each others center of mass.
 */
public class DistanceConstraint extends Constraint {
    private double distance;                // The distance to keep , If undefined, it will be set to the current distance between bodyA and bodyB
    private ContactEquation distanceEquation; // Distance equation for the constraint

    /**
     * Creates a new DistanceConstraint with natural length as current distance between objects A and B 
     * @param bodyA
     * @param bodyB
     */
    public DistanceConstraint(Body bodyA, Body bodyB){
         this(bodyA, bodyB,bodyA.position.distanceTo(bodyB.position),1e6);
    }
    /**
   * @param distance The distance to keep must be positive
   * @param maxForce The maximum force that should be applied to constrain the bodies.
   */
    public DistanceConstraint(Body bodyA, Body bodyB, double distance, double maxForce) {
        super(bodyA, bodyB);
        this.distance = distance;
        this.distanceEquation = new ContactEquation(bodyA, bodyB);
        this.equations.add(this.distanceEquation);

        // Set the minimum and maximum forces for the distance equation to achieve the constraint
        this.distanceEquation.minForce = -maxForce;
        this.distanceEquation.maxForce = maxForce;
    }

    public void update() {
        Body bodyA = this.bodyA;
        Body bodyB = this.bodyB;
        ContactEquation eq = this.distanceEquation;
        double halfDist = this.distance * 0.5;
        Vec3 normal = eq.ni;

        // Calculate the direction from bodyA to bodyB and normalize it
        bodyB.position.vsub(bodyA.position, normal);
        normal.normalize();

        // Set the contact points relative to the bodies
        normal.scale(halfDist, eq.ri); // Offset from the center of bodyA
        normal.scale(-halfDist, eq.rj); // Offset from the center of bodyB
    }
}
