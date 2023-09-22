package constraints;

import math.Vec3;
import equations.RotationalEquation;
import equations.RotationalEquationOptions;
import equations.RotationalMotorEquation;
import objects.Body;

/**
 * Lock constraint. Removes all degrees of freedom between the bodies.
 */
public class LockConstraint extends PointToPointConstraint {
    // Public properties for easy access
    public Vec3 xA;
    public Vec3 xB;
    public Vec3 yA;
    public Vec3 yB;
    public Vec3 zA;
    public Vec3 zB;
    public RotationalEquation rotationalEquation1;
    public RotationalEquation rotationalEquation2;
    public RotationalEquation rotationalEquation3;
    public RotationalMotorEquation motorEquation;
    
    
 // Temporary vectors used for calculations
    public static Vec3 LockConstraint_update_tmpVec1 = new Vec3();
    public static Vec3 LockConstraint_update_tmpVec2 = new Vec3();
    
    
    public LockConstraint(Body bodyA, Body bodyB) {
        this(bodyA, bodyB,new LockConstraintOptions(bodyA, bodyB ));
    }

    /**
     * Constructor for the LockConstraint.
     * @param bodyA The first body.
     * @param bodyB The second body.
     * @param options Constraint options.
     */
    public LockConstraint(Body bodyA, Body bodyB, LockConstraintOptions options) {
        //double maxForce = (options.maxForce != null) ? options.maxForce : 1e6;

        // The point-to-point constraint will keep a point shared between the bodies
        //changed pivotA and PivotB to new Vec3() in the constructor  and maxForce with options.maxForce
        super(bodyA,bodyA.pointToLocalFrame(bodyA.position.vadd(bodyB.position,new Vec3()).scale(0.5,bodyA.position.vadd(bodyB.position,new Vec3())), new Vec3()) , bodyB, bodyB.pointToLocalFrame(bodyA.position.vadd(bodyB.position,new Vec3()).scale(0.5,bodyA.position.vadd(bodyB.position,new Vec3())), new Vec3()), options.maxForce);


        //added : shifted the  upper portion after the super constructor 
        //double maxForce = options.maxForce;
         // Set pivot point in between
        Vec3 pivotA = new Vec3();
        Vec3 pivotB = new Vec3();
        Vec3 halfWay = new Vec3();
        bodyA.position.vadd(bodyB.position, halfWay);
        halfWay.scale(0.5, halfWay);
        bodyB.pointToLocalFrame(halfWay, pivotB);
        bodyA.pointToLocalFrame(halfWay, pivotA);

        //added : copied pivotA in this.pivotA similar for pivotB 
        this.pivotA.copy(pivotA);
        this.pivotB.copy(pivotB);

        // Store initial rotation of the bodies as unit vectors in the local body spaces
        this.xA = bodyA.vectorToLocalFrame(Vec3.UNIT_X.clone());
        this.xB = bodyB.vectorToLocalFrame(Vec3.UNIT_X.clone());
        this.yA = bodyA.vectorToLocalFrame(Vec3.UNIT_Y.clone());
        this.yB = bodyB.vectorToLocalFrame(Vec3.UNIT_Y.clone());
        this.zA = bodyA.vectorToLocalFrame(Vec3.UNIT_Z.clone());
        this.zB = bodyB.vectorToLocalFrame(Vec3.UNIT_Z.clone());

        //added rotational equation
        RotationalEquationOptions rotationalOptions = new RotationalEquationOptions(new Vec3(1, 0, 0),new Vec3(0, 1, 0),Math.PI / 2,options.maxForce);

        // ...and the following rotational equations will keep all rotational DOF's in place
        this.rotationalEquation1 = new RotationalEquation(bodyA, bodyB, rotationalOptions);
        this.rotationalEquation2 = new RotationalEquation(bodyA, bodyB, rotationalOptions);
        this.rotationalEquation3 = new RotationalEquation(bodyA, bodyB, rotationalOptions);

        this.equations.add(this.rotationalEquation1);
        this.equations.add(this.rotationalEquation2);
        this.equations.add(this.rotationalEquation3);
    }

    /**
     * Updates the lock constraint.
     */
    public void update() {
        Body bodyA = this.bodyA;
        Body bodyB = this.bodyB;
        RotationalMotorEquation motor = this.motorEquation;
        RotationalEquation r1 = this.rotationalEquation1;
        RotationalEquation r2 = this.rotationalEquation2;
        RotationalEquation r3 = this.rotationalEquation3;
        Vec3 worldAxisA = LockConstraint_update_tmpVec1;
        Vec3 worldAxisB = LockConstraint_update_tmpVec2;

        super.update();

        // These vector pairs must be orthogonal
        bodyA.vectorToWorldFrame(this.xA, r1.axisA);
        bodyB.vectorToWorldFrame(this.yB, r1.axisB);

        bodyA.vectorToWorldFrame(this.yA, r2.axisA);
        bodyB.vectorToWorldFrame(this.zB, r2.axisB);

        bodyA.vectorToWorldFrame(this.zA, r3.axisA);
        bodyB.vectorToWorldFrame(this.xB, r3.axisB);
    }
}


