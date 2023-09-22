package constraints;

import math.Vec3;
import equations.ContactEquation;
import objects.Body;

/**
 * Connects two bodies at given offset points.
 * @example
 *     const bodyA = new Body({ mass: 1 })
 *     const bodyB = new Body({ mass: 1 })
 *     bodyA.position.set(-1, 0, 0)
 *     bodyB.position.set(1, 0, 0)
 *     bodyA.addShape(shapeA)
 *     bodyB.addShape(shapeB)
 *     world.addBody(bodyA)
 *     world.addBody(bodyB)
 *     const localPivotA = new Vec3(1, 0, 0)
 *     const localPivotB = new Vec3(-1, 0, 0)
 *     const constraint = new PointToPointConstraint(bodyA, localPivotA, bodyB, localPivotB)
 *     world.addConstraint(constraint)
 */
public class PointToPointConstraint extends Constraint {
    /**
   * Pivot, defined locally in bodyA.
   */
    protected Vec3 pivotA;
    /**
   * Pivot, defined locally in bodyB.
   */
    protected Vec3 pivotB;

    private ContactEquation equationX;
    private ContactEquation equationY;
    private ContactEquation equationZ;

    /**
   * @param pivotA The point relative to the center of mass of bodyA which bodyA is constrained to.
   * @param bodyB Body that will be constrained in a similar way to the same point as bodyA. We will therefore get a link between bodyA and bodyB. If not specified, bodyA will be constrained to a static point.
   * @param pivotB The point relative to the center of mass of bodyB which bodyB is constrained to.
   * @param maxForce The maximum force that should be applied to constrain the bodies.
   */
    public PointToPointConstraint(Body bodyA, Vec3 pivotA, Body bodyB, Vec3 pivotB, double maxForce) {
        super(bodyA, bodyB);

        // Initialize local pivot points
        this.pivotA = pivotA.clone();
        this.pivotB = pivotB.clone();

        // Create contact equations for X, Y, and Z axes
        this.equationX = new ContactEquation(bodyA, bodyB);
        this.equationY = new ContactEquation(bodyA, bodyB);
        this.equationZ = new ContactEquation(bodyA, bodyB);

        // Equations to be fed to the solver
        this.equations.add(this.equationX);
        this.equations.add(this.equationY);
        this.equations.add(this.equationZ);

        // Make the equations bidirectional with specified maximum force
        this.equationX.minForce = this.equationY.minForce = this.equationZ.minForce = -maxForce;
        this.equationX.maxForce = this.equationY.maxForce = this.equationZ.maxForce = maxForce;

        // Set the normals for X, Y, and Z axes
        this.equationX.ni.set(1, 0, 0);
        this.equationY.ni.set(0, 1, 0);
        this.equationZ.ni.set(0, 0, 1);
    }

    public void update() {
        Body bodyA = this.bodyA;
        Body bodyB = this.bodyB;

        // Rotate the local pivot points to world space
        bodyA.quaternion.vmult(this.pivotA, this.equationX.ri);
        bodyB.quaternion.vmult(this.pivotB, this.equationX.rj);

        // Copy the rotated pivot points to Y and Z equations
        this.equationY.ri.copy(this.equationX.ri);
        this.equationY.rj.copy(this.equationX.rj);
        this.equationZ.ri.copy(this.equationX.ri);
        this.equationZ.rj.copy(this.equationX.rj);
    }
}
