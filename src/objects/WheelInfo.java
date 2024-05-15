package objects;

import math.Transform;
import math.Vec3;

public class WheelInfo {
    /**
   * Max travel distance of the suspension, in meters.
   * @default 1
   */
	public static Vec3 relpos = new Vec3();
	public static Vec3 chassis_velocity_at_contactPoint = new Vec3() ;
    public double maxSuspensionTravel;
    /**
   * Speed to apply to the wheel rotation when the wheel is sliding.
   * @default -0.1
   */
    double customSlidingRotationalSpeed;
    /**
   * If the customSlidingRotationalSpeed should be used.
   * @default false
   */
    boolean useCustomSlidingRotationalSpeed;
    boolean sliding;
    /**
   * Connection point, defined locally in the chassis body frame.
   */
    Vec3 chassisConnectionPointLocal;
    Vec3 chassisConnectionPointWorld;
    Vec3 directionLocal;
    Vec3 directionWorld;
    Vec3 axleLocal;
    Vec3 axleWorld;
    double suspensionRestLength;
    /**
   * suspensionMaxLength
   * @default 2
   */
    private double suspensionMaxLength;
    /**
   * radius
   * @default 1
   */
    double radius;
    /**
   * suspensionStiffness
   * @default 100
   */
    double suspensionStiffness;
    /**
   * dampingCompression
   * @default 10
   */
    double dampingCompression;
    /**
   * dampingRelaxation
   * @default 10
   */
    double dampingRelaxation;
    /**
   * frictionSlip
   * @default 10.5
   */
    protected double frictionSlip;
    protected double forwardAcceleration;
    protected double sideAcceleration;
    /**
   * steering
   * @default 0
   */
    double steering;
    /**
   * Rotation value, in radians.
   * @default 0
   */
    double rotation;
    /**
   * deltaRotation
   * @default 0
   */
    double deltaRotation;
    /**
   * rollInfluence
   * @default 0.01
   */
    protected double rollInfluence;
    double maxSuspensionForce;
    double engineForce;
    double brake;
    /**
   * isFrontWheel
   * @default true
   */
    private boolean isFrontWheel;
    /**
   * clippedInvContactDotSuspension
   * @default 1
   */
    double clippedInvContactDotSuspension;
    /**
   * suspensionRelativeVelocity
   * @default 0
   */
    double suspensionRelativeVelocity;
    /**
   * suspensionForce
   * @default 0
   */
    double suspensionForce;

    protected double slipInfo;
    /**
   * skidInfo
   * @default 0
   */
    protected double skidInfo;
    /**
   * suspensionLength
   * @default 0
   */
    protected double suspensionLength;
    protected double sideImpulse;
    protected double forwardImpulse;
    /**
   * The result from raycasting.
   */
    WheelRaycastResult raycastResult;
    Transform worldTransform;
    boolean isInContact;

    public WheelInfo() {
        this.maxSuspensionTravel = 1.0;
        this.customSlidingRotationalSpeed = -0.1;
        this.useCustomSlidingRotationalSpeed = false;
        this.sliding = false;
        this.chassisConnectionPointLocal = new Vec3();
        this.chassisConnectionPointWorld = new Vec3();
        this.directionLocal = new Vec3();
        this.directionWorld = new Vec3();
        this.axleLocal = new Vec3();
        this.axleWorld = new Vec3();
        this.suspensionRestLength = 1.0;
        this.suspensionMaxLength = 2.0;
        this.radius = 1.0;
        this.suspensionStiffness = 100.0;
        this.dampingCompression = 10.0;
        this.dampingRelaxation = 10.0;
        this.frictionSlip = 10.5;
        this.forwardAcceleration = 1.0;
        this.sideAcceleration = 1.0;
        this.steering = 0.0;
        this.rotation = 0.0;
        this.deltaRotation = 0.0;
        this.rollInfluence = 0.01;
        this.maxSuspensionForce = Double.MAX_VALUE;
        this.isFrontWheel = true;
        this.clippedInvContactDotSuspension = 1.0;
        this.suspensionRelativeVelocity = 0.0;
        this.suspensionForce = 0.0;
        this.slipInfo = 0.0;
        this.skidInfo = 0.0;
        this.suspensionLength = 0.0;
        this.sideImpulse = 0.0;
        this.forwardImpulse = 0.0;
        this.raycastResult = new WheelRaycastResult();
        this.worldTransform = new Transform();
        this.isInContact = false;
    }

    public WheelInfo(WheelInfoOptions options) {
        //options = Utils.defaults(options, new WheelInfoOptions());
        
        // Initialize fields based on options here
        
        this.maxSuspensionTravel = options.maxSuspensionTravel;
        	    this.customSlidingRotationalSpeed = options.customSlidingRotationalSpeed;
        	    this.useCustomSlidingRotationalSpeed = options.useCustomSlidingRotationalSpeed;
        	    this.sliding = false;
        	    this.chassisConnectionPointLocal = options.chassisConnectionPointLocal.clone() ;
        	    this.chassisConnectionPointWorld = options.chassisConnectionPointWorld.clone() ;
        	    this.directionLocal = options.directionLocal.clone();
        	    this.directionWorld = options.directionWorld.clone();
        	    this.axleLocal = options.axleLocal.clone();
        	    this.axleWorld = options.axleWorld.clone();
        	    this.suspensionRestLength = options.suspensionRestLength;
        	    this.suspensionMaxLength = options.suspensionMaxLength;
        	    this.radius = options.radius;
        	    this.suspensionStiffness = options.suspensionStiffness;
        	    this.dampingCompression = options.dampingCompression;
        	    this.dampingRelaxation = options.dampingRelaxation;
        	    this.frictionSlip = options.frictionSlip;
        	    this.forwardAcceleration = options.forwardAcceleration;
        	    this.sideAcceleration = options.sideAcceleration;
        	    this.steering = 0;
        	    this.rotation = 0;
        	    this.deltaRotation = 0;
        	    this.rollInfluence = options.rollInfluence;
        	    this.maxSuspensionForce = options.maxSuspensionForce;
        	    this.engineForce = 0;
        	    this.brake = 0;
        	    this.isFrontWheel = options.isFrontWheel;
        	    this.clippedInvContactDotSuspension = 1;
        	    this.suspensionRelativeVelocity = 0;
        	    this.suspensionForce = 0;
        	    this.slipInfo = 0;
        	    this.skidInfo = 0;
        	    this.suspensionLength = 0;
        	    this.sideImpulse = 0;
        	    this.forwardImpulse = 0;
        	    this.raycastResult = new WheelRaycastResult();
        	    this.worldTransform = new Transform();
        	    this.isInContact = false;
        	  
    }

    public void updateWheel(Body chassis) {
        WheelRaycastResult raycastResult = this.raycastResult;

        if (this.isInContact) {
            double project = raycastResult.hitNormalWorld.dot(raycastResult.getDirectionWorld());
            raycastResult.hitPointWorld.vsub(chassis.position, relpos);
            chassis.getVelocityAtWorldPoint(relpos, chassis_velocity_at_contactPoint);
            double projVel = raycastResult.hitNormalWorld.dot(chassis_velocity_at_contactPoint);
            if (project >= -0.1) {
                this.suspensionRelativeVelocity = 0.0;
                this.clippedInvContactDotSuspension = 1.0 / 0.1;
            } else {
                double inv = -1.0 / project;
                this.suspensionRelativeVelocity = projVel * inv;
                this.clippedInvContactDotSuspension = inv;
            }
        } else {
            raycastResult.setSuspensionLength(this.suspensionRestLength);
            this.suspensionRelativeVelocity = 0.0;
            raycastResult.getDirectionWorld().scale(-1.0, raycastResult.hitNormalWorld);
            this.clippedInvContactDotSuspension = 1.0;
        }
    }
    
    // Add getters and setters for other fields as needed.
}

