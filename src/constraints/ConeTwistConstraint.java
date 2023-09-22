package constraints;

import math.Vec3;
import equations.*;
import objects.Body;

/**
 * A Cone Twist constraint, useful for ragdolls.
 */
public class ConeTwistConstraint extends PointToPointConstraint {
    private Vec3 axisA;                  // The axis direction for the constraint of body A
    private Vec3 axisB;                  // The axis direction for the constraint of body B
    private double angle;                // The aperture angle of the cone
    private double twistAngle;           // The twist angle of the joint
    private ConeEquation coneEquation;    // Cone equation for the constraint
    private RotationalEquation twistEquation; // Twist equation for the constraint
    
    public ConeTwistConstraint(Body bodyA, Body bodyB) {
    	this(bodyA,bodyB,new ConeTwistConstraintOptions());
    }
    /*
     * @param pivotA : The pivot point for bodyA.
     * @param pivotB : The pivot point for bodyB.
     * @param axisA : The axis direction for the constraint of the body A.
     * @param axisB : The axis direction for the constraint of the body B.
     * @param angle : The aperture angle of the cone. @default : 0.0
     * @param twistAngle : The twist angle of the joint , @default : 0.0
     * @param maxForce : The maximum force that should be applied to constrain the bodies. @default : 1e6
     * @param collideConnected : Whether to collide the connected bodies or not . @default false  
     */
    public ConeTwistConstraint(Body bodyA, Body bodyB, ConeTwistConstraintOptions options) {

        // Set pivot point in between
        //Vec3 pivotA = (options.pivotA != null) ? options.pivotA.clone() : new Vec3();
        //Vec3 pivotB = (options.pivotB != null) ? options.pivotB.clone() : new Vec3();

        super(bodyA, options.pivotA, bodyB, options.pivotB, options.maxForce);
        
        // changed : shifted pivotA and pivotB after super construction
        //Vec3 pivotA = (options.pivotA != null) ? options.pivotA.clone() : new Vec3();
        //Vec3 pivotB = (options.pivotB != null) ? options.pivotB.clone() : new Vec3();
        //double maxForce = 1e6 ; //(options.maxForce != null) ? options.maxForce : 1e6;
        //added workaround for the above problem
        double maxForce = options.maxForce ;
        this.axisA = (options.axisA != null) ? options.axisA.clone() : new Vec3();
        this.axisB = (options.axisB != null) ? options.axisB.clone() : new Vec3();

        this.collideConnected = options.collideConnected ; //false ; //(options.collideConnected != null) ? options.collideConnected : false;

        //this.angle = 0 ; //(options.angle != null) ? options.angle : 0;
        //added 
        this.angle =  options.angle ;
        /*
         * additional code to convert conetwistconstraintoptions to coneequationOptions and rotationalequationoptions 
         * 
         * in rotationalEquationOptions options.angle for options.maxAngle 
         */
        
        ConeEquationOptions coneOptions = new ConeEquationOptions(options.maxForce,options.axisA,options.axisB,options.angle);
        RotationalEquationOptions rotationalOptions = new RotationalEquationOptions(options.axisA , options.axisB , options.angle , options.maxForce);

        this.coneEquation = new ConeEquation(bodyA, bodyB, coneOptions);

        this.twistEquation = new RotationalEquation(bodyA, bodyB, rotationalOptions);
        //this.twistAngle = 0 ; //(options.twistAngle != null) ? options.twistAngle : 0;
        //added 
        this.twistAngle = Double.isNaN(options.twistAngle) ? 0 : options.twistAngle ; 
        // Make the cone equation push the bodies toward the cone axis, not outward
        coneEquation.maxForce = 0;
        coneEquation.minForce = -maxForce;

        // Make the twist equation add torque toward the initial position
        twistEquation.maxForce = 0;
        twistEquation.minForce = -maxForce;

        this.equations.add(coneEquation);
        this.equations.add(twistEquation);
    }

    public void update() {
        super.update();
        
        Body bodyA = this.bodyA;
        Body bodyB = this.bodyB;
        ConeEquation cone = this.coneEquation;
        RotationalEquation twist = this.twistEquation;

        // Update the axes to the cone constraint
        bodyA.vectorToWorldFrame(this.axisA, cone.axisA);
        bodyB.vectorToWorldFrame(this.axisB, cone.axisB);

        // Update the world axes in the twist constraint
        this.axisA.tangents(twist.axisA, twist.axisA);
        bodyA.vectorToWorldFrame(twist.axisA, twist.axisA);

        this.axisB.tangents(twist.axisB, twist.axisB);
        bodyB.vectorToWorldFrame(twist.axisB, twist.axisB);

        cone.angle = this.angle;
        twist.maxAngle = this.twistAngle;
    }
}
