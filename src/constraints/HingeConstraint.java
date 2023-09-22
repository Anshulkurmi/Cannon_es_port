package constraints;

import math.Vec3;
import equations.RotationalEquation;
import equations.RotationalEquationOptions;
import equations.RotationalMotorEquation;
import objects.Body;

/**
 * Hinge constraint. Think of it as a door hinge. It tries to keep the door in the correct place and with the correct orientation.
 */
public class HingeConstraint extends PointToPointConstraint {
    private Vec3 axisA; // Rotation axis, defined locally in bodyA
    private Vec3 axisB; // Rotation axis, defined locally in bodyB
    private RotationalEquation rotationalEquation1;
    private RotationalEquation rotationalEquation2;
    private RotationalMotorEquation motorEquation;
    
 // Temporary vectors used for calculations
    public static Vec3 HingeConstraint_update_tmpVec1 = new Vec3();
    public static Vec3 HingeConstraint_update_tmpVec2 = new Vec3();
    
    /**
     * Creates Hinge with default settings
     * @param bodyA
     * @param bodyB
     */
    public HingeConstraint(Body bodyA, Body bodyB) {
    	this(bodyA,bodyB,new HingeConstraintOptions(bodyA,bodyB));
    }

    /**
     * Constructor for the HingeConstraint.
     * @param bodyA The first body.
     * @param bodyB The second body.
     * @param options Constraint options.
     * @param options.pivotA : A point defined locally in bodyA. This defines the offset of axisA.
     * @param options.pivotB : A point defined locally in bodyB. This defines the offset of axisB.
     * @param options.axisA :  An axis that bodyA can rotate around, defined locally in bodyB.
     * @param options.axisB :  An axis that bodyB can rotate around, defined locally in bodyB.
     * @param collideConnected  : Wheter to collide the connected bodies or not. @default false.
     * @param maxForce : The maximum force that should be applied to constrain the bodies. @default 1e6
     */
    public HingeConstraint(Body bodyA, Body bodyB, HingeConstraintOptions options) {
        //double maxForce = (options.maxForce != null) ? options.maxForce : 1e6;
        //Vec3 pivotA = (options.pivotA != null) ? options.pivotA.clone() : new Vec3();
        //Vec3 pivotB = (options.pivotB != null) ? options.pivotB.clone() : new Vec3();

        super(bodyA, options.pivotA, bodyB, options.pivotB, options.maxForce);

        this.axisA = (options.axisA != null) ? options.axisA.clone() : new Vec3(1, 0, 0);
        this.axisA.normalize();

        this.axisB = (options.axisB != null) ? options.axisB.clone() : new Vec3(1, 0, 0);
        this.axisB.normalize();

        this.collideConnected = false ;//(options.collideConnected != null) ? options.collideConnected : false;
        
        //added ... converting options of type HingeConstraingOptions to RotationalEquationOptions
        RotationalEquationOptions rotationalOptions = new RotationalEquationOptions(options.axisA,options.axisB,Math.PI / 2,options.maxForce);
        this.rotationalEquation1 = new RotationalEquation(bodyA, bodyB, rotationalOptions);
        this.rotationalEquation2 = new RotationalEquation(bodyA, bodyB, rotationalOptions);
        this.motorEquation = new RotationalMotorEquation(bodyA, bodyB, options.maxForce);
        this.motorEquation.enabled =  false; // Not enabled by default

        // Equations to be fed to the solver
        this.equations.add(this.rotationalEquation1);
        this.equations.add(this.rotationalEquation2);
        this.equations.add(this.motorEquation);
    }

    /**
     * Enables the motor for the hinge constraint.
     */
    public void enableMotor() {
        this.motorEquation.enabled = true;
    }

    /**
     * Disables the motor for the hinge constraint.
     */
    public void disableMotor() {
        this.motorEquation.enabled =  false;
    }

    /**
     * Sets the motor speed for the hinge constraint.
     * @param speed The motor speed.
     */
    public void setMotorSpeed(double speed) {
        this.motorEquation.targetVelocity = speed;
    }

    /**
     * Sets the maximum motor force for the hinge constraint.
     * @param maxForce The maximum motor force.
     */
    public void setMotorMaxForce(double maxForce) {
        this.motorEquation.maxForce = maxForce ;
        this.motorEquation.minForce = -maxForce;
    }

    /**
     * Updates the hinge constraint.
     */
    public void update() {
        Body bodyA = this.bodyA;
        Body bodyB = this.bodyB;
        RotationalMotorEquation motor = this.motorEquation;
        RotationalEquation r1 = this.rotationalEquation1;
        RotationalEquation r2 = this.rotationalEquation2;
        Vec3 worldAxisA = HingeConstraint_update_tmpVec1;
        Vec3 worldAxisB = HingeConstraint_update_tmpVec2;

        Vec3 axisA = this.axisA;
        Vec3 axisB = this.axisB;

        super.update();

        // Get world axes
        bodyA.quaternion.vmult(axisA, worldAxisA);
        bodyB.quaternion.vmult(axisB, worldAxisB);

        worldAxisA.tangents(r1.axisA , r2.axisA );
        r1.axisB = worldAxisB.clone();
        r2.axisB = worldAxisB.clone();

        if (this.motorEquation.enabled) {
            bodyA.quaternion.vmult(this.axisA, motor.axisA);
            bodyB.quaternion.vmult(this.axisB, motor.axisB);
        }
    }
}


