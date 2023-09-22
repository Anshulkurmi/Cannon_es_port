package objects;

import math.Vec3;

public class WheelInfoOptions {
    /**
       * Connection point, defined locally in the chassis body frame.
       */
    public Vec3 chassisConnectionPointLocal;
    public Vec3 chassisConnectionPointWorld;
    public Vec3 directionLocal;
    public Vec3 directionWorld;
    public Vec3 axleLocal;
    public Vec3 axleWorld;
    /**
       * suspensionRestLength
       * @default 1
    */
    public double suspensionRestLength;
    /**
       * suspensionMaxLength
       * @default 2
       */
    public double suspensionMaxLength;
    /**
       * radius
       * @default 1
    */
    public double radius;
    /**
       * suspensionStiffness
       * @default 100
    */
    public double suspensionStiffness;
    /**
       * dampingCompression
       * @default 10
    */
    public double dampingCompression;
    /**
       * dampingRelaxation
       * @default 10
    */
    public double dampingRelaxation;
    /**
       * frictionSlip
       * @default 10.5
       */
    public double frictionSlip;
    public double forwardAcceleration;
    public double sideAcceleration;
    /**
       * steering
       * @default 0
    */
    public double steering;
     /**
       * Rotation value, in radians.
       * @default 0
    */
    public double rotation;
    /**
       * deltaRotation
       * @default 0
       */
    public double deltaRotation;
    /**
       * rollInfluence
       * @default 0.01
       */
    public double rollInfluence;
    public double maxSuspensionForce;
    /**
       * isFrontWheel
       * @default true
       */
    public boolean isFrontWheel;
    /**
       * clippedInvContactDotSuspension
       * @default 1
       */
    public double clippedInvContactDotSuspension;
    /**
       * suspensionRelativeVelocity
       * @default 0
       */
    public double suspensionRelativeVelocity;
    /**
       * suspensionForce
       * @default 0
       */
    public double suspensionForce;
    public double slipInfo;
    /**
       * skidInfo
       * @default 0
       */
    public double skidInfo;
    public double suspensionLength;
    /**
       * Max travel distance of the suspension, in meters.
       * @default 1
       */
    public double maxSuspensionTravel;
    /**
       * If the customSlidingRotationalSpeed should be used.
       * @default false
       */
    public boolean useCustomSlidingRotationalSpeed;
    /**
       * Speed to apply to the wheel rotation when the wheel is sliding.
       * @default -0.1
       */
    public double customSlidingRotationalSpeed;
    
    // Constructors for default values and custom options
    public WheelInfoOptions() {
        // Initialize default values here
    }
    
    public WheelInfoOptions(Vec3 chassisConnectionPointLocal, Vec3 chassisConnectionPointWorld, Vec3 directionLocal, Vec3 directionWorld, 
                            Vec3 axleLocal, Vec3 axleWorld, double suspensionRestLength, double suspensionMaxLength, double radius, 
                            double suspensionStiffness, double dampingCompression, double dampingRelaxation, double frictionSlip, 
                            double forwardAcceleration, double sideAcceleration, double steering, double rotation, double deltaRotation, 
                            double rollInfluence, double maxSuspensionForce, boolean isFrontWheel, double clippedInvContactDotSuspension, 
                            double suspensionRelativeVelocity, double suspensionForce, double slipInfo, double skidInfo, 
                            double suspensionLength, double maxSuspensionTravel, boolean useCustomSlidingRotationalSpeed, 
                            double customSlidingRotationalSpeed) {
        this.chassisConnectionPointLocal = chassisConnectionPointLocal;
        this.chassisConnectionPointWorld = chassisConnectionPointWorld;
        this.directionLocal = directionLocal;
        this.directionWorld = directionWorld;
        this.axleLocal = axleLocal;
        this.axleWorld = axleWorld;
        this.suspensionRestLength = suspensionRestLength;
        this.suspensionMaxLength = suspensionMaxLength;
        this.radius = radius;
        this.suspensionStiffness = suspensionStiffness;
        this.dampingCompression = dampingCompression;
        this.dampingRelaxation = dampingRelaxation;
        this.frictionSlip = frictionSlip;
        this.forwardAcceleration = forwardAcceleration;
        this.sideAcceleration = sideAcceleration;
        this.steering = steering;
        this.rotation = rotation;
        this.deltaRotation = deltaRotation;
        this.rollInfluence = rollInfluence;
        this.maxSuspensionForce = maxSuspensionForce;
        this.isFrontWheel = isFrontWheel;
        this.clippedInvContactDotSuspension = clippedInvContactDotSuspension;
        this.suspensionRelativeVelocity = suspensionRelativeVelocity;
        this.suspensionForce = suspensionForce;
        this.slipInfo = slipInfo;
        this.skidInfo = skidInfo;
        this.suspensionLength = suspensionLength;
        this.maxSuspensionTravel = maxSuspensionTravel;
        this.useCustomSlidingRotationalSpeed = useCustomSlidingRotationalSpeed;
        this.customSlidingRotationalSpeed = customSlidingRotationalSpeed;
    }
}
