package objects;

import java.util.ArrayList;
import java.util.List;

import collision.Ray;
import collision.RaycastResult;
import constraints.Constraint;
import math.Quaternion;
import math.Transform;
import math.Vec3;
import world.World;

public class RaycastVehicle {
    private Body chassisBody; // The car chassis body.
    private List<WheelInfo> wheelInfos; // List to store information about each wheel.
    private boolean sliding; // Indicates if the car is sliding.
    private World world; // Reference to the physics world.
    private int indexRightAxis; // Index of the right axis (x=0, y=1, z=2).
    private int indexForwardAxis; // Index of the forward axis (x=0, y=1, z=2).
    private int indexUpAxis; // Index of the up axis (x=0, y=1, z=2).
    private List<Constraint> constraints; // List to store constraints.
    private Runnable preStepCallback; // Optional pre-step callback.
    private double currentVehicleSpeedKmHour; // Current vehicle speed in kilometers per hour.
    private int numWheelsOnGround; // Number of wheels on the ground.

    // Constructor to initialize the RaycastVehicle.
    public RaycastVehicle(Body chassisBody, int indexRightAxis, int indexForwardAxis, int indexUpAxis) {
        this.chassisBody = chassisBody;
        this.wheelInfos = new ArrayList<>();
        this.sliding = false;
        this.world = null;
        this.indexRightAxis = indexRightAxis;
        this.indexForwardAxis = indexForwardAxis;
        this.indexUpAxis = indexUpAxis;
        this.constraints = new ArrayList<>();
        this.preStepCallback = () -> {
        };
        this.currentVehicleSpeedKmHour = 0;
        this.numWheelsOnGround = 0;
    }

    // Method to add a wheel to the vehicle.
    public int addWheel(WheelInfoOptions options) {
        WheelInfo info = new WheelInfo(options);
        int index = this.wheelInfos.size();
        this.wheelInfos.add(info);
        return index;
    }

    // Method to set the steering value of a wheel.
    public void setSteeringValue(double value, int wheelIndex) {
        wheelInfos.get(wheelIndex).steering = value;
    }

    // Method to apply engine force to a wheel.
    public void applyEngineForce(double value, int wheelIndex) {
        this.wheelInfos.get(wheelIndex).engineForce = value;
    }

    // Method to set the braking force of a wheel.
    public void setBrake(double brake, int wheelIndex) {
        this.wheelInfos.get(wheelIndex).brake=brake;
    }

    // Method to add the vehicle, including its constraints, to the physics world.
    public void addToWorld(World world) {
        world.addBody(chassisBody);
        this.world = world;
        preStepCallback = () -> updateVehicle(world.dt);
        world.addEventListener("prestep",null);
    }

    // Method to get the world-oriented axis for the vehicle.
    private void getVehicleAxisWorld(int axisIndex, Vec3 result) {
        result.set(axisIndex == 0 ? 1.0 : 0.0, axisIndex == 1 ? 1.0 : 0.0, axisIndex == 2 ? 1.0 : 0.0);
        chassisBody.vectorToWorldFrame(result, result);
    }

    // Method to update the vehicle's state and behavior based on the current time
    // step.
    public void updateVehicle(double timeStep) {
        List<WheelInfo> wheelInfos = this.wheelInfos;
        int numWheels = wheelInfos.size();
        Body chassisBody = this.chassisBody;

        for (int i = 0; i < numWheels; i++) {
            updateWheelTransform(i);
        }

        this.currentVehicleSpeedKmHour = 3.6 * chassisBody.velocity.length();

        Vec3 forwardWorld = new Vec3();
        getVehicleAxisWorld(this.indexForwardAxis, forwardWorld);

        if (forwardWorld.dot(chassisBody.velocity) < 0) {
            this.currentVehicleSpeedKmHour *= -1;
        }

        // Simulate suspension
        for (int i = 0; i < numWheels; i++) {
            castRay(wheelInfos.get(i));
        }

        updateSuspension(timeStep);

        Vec3 impulse = new Vec3();
        Vec3 relpos = new Vec3();
        for (int i = 0; i < numWheels; i++) {
            // Apply suspension force
            WheelInfo wheel = wheelInfos.get(i);
            double suspensionForce = wheel.suspensionForce;
            if (suspensionForce > wheel.maxSuspensionForce) {
                suspensionForce = wheel.maxSuspensionForce;
            }
            wheel.raycastResult.hitNormalWorld.scale(suspensionForce * timeStep, impulse);

            wheel.raycastResult.hitPointWorld.vsub(chassisBody.position, relpos);
            chassisBody.applyImpulse(impulse, relpos);
        }

        updateFriction(timeStep);

        Vec3 hitNormalWorldScaledWithProj = new Vec3();
        Vec3 fwd = new Vec3();
        Vec3 vel = new Vec3();
        for (int i = 0; i < numWheels; i++) {
            WheelInfo wheel = wheelInfos.get(i);
            // Vec3 relpos = new Vec3();
            // wheel.chassisConnectionPointWorld.vsub(chassisBody.position, relpos);
            chassisBody.getVelocityAtWorldPoint(wheel.chassisConnectionPointWorld, vel);

            // Hack to get the rotation in the correct direction
            int m = 1;
            switch (this.indexUpAxis) {
                case 1:
                    m = -1;
                    break;
            }

            if (wheel.isInContact) {
                getVehicleAxisWorld(this.indexForwardAxis, fwd);
                double proj = fwd.dot(wheel.raycastResult.hitNormalWorld);
                wheel.raycastResult.hitNormalWorld.scale(proj, hitNormalWorldScaledWithProj);

                fwd.vsub(hitNormalWorldScaledWithProj, fwd);

                double proj2 = fwd.dot(vel);
                wheel.deltaRotation = (m * proj2 * timeStep) / wheel.radius;
            }

            if ((wheel.sliding || !wheel.isInContact) && wheel.engineForce != 0
                    && wheel.useCustomSlidingRotationalSpeed) {
                // Apply custom rotation when accelerating and sliding
                wheel.deltaRotation = (wheel.engineForce > 0 ? 1 : -1) * wheel.customSlidingRotationalSpeed * timeStep;
            }

            // Lock wheels
            if (Math.abs(wheel.brake) > Math.abs(wheel.engineForce)) {
                wheel.deltaRotation = 0;
            }

            wheel.rotation += wheel.deltaRotation; // Use the old value
            wheel.deltaRotation *= 0.99; // Damping of rotation when not in contact
        }
    }

    public void updateSuspension(double deltaTime) {
        Body chassisBody = this.chassisBody;
        double chassisMass = chassisBody.mass;
        List<WheelInfo> wheelInfos = this.wheelInfos;
        int numWheels = wheelInfos.size();

        for (int w_it = 0; w_it < numWheels; w_it++) {
            WheelInfo wheel = wheelInfos.get(w_it);

            if (wheel.isInContact) {
                double force;

                // Spring
                double susp_length = wheel.suspensionRestLength;
                double current_length = wheel.suspensionLength;
                double length_diff = susp_length - current_length;

                force = wheel.suspensionStiffness * length_diff * wheel.clippedInvContactDotSuspension;

                // Damper
                double projected_rel_vel = wheel.suspensionRelativeVelocity;
                double susp_damping;
                if (projected_rel_vel < 0) {
                    susp_damping = wheel.dampingCompression;
                } else {
                    susp_damping = wheel.dampingRelaxation;
                }
                force -= susp_damping * projected_rel_vel;

                wheel.suspensionForce = force * chassisMass;
                if (wheel.suspensionForce < 0) {
                    wheel.suspensionForce = 0;
                }
            } else {
                wheel.suspensionForce = 0;
            }
        }
    }

    /**
     * Remove the vehicle including its constraints from the world.
     */
    public void removeFromWorld(World world) {
        List<Constraint> constraints = this.constraints;
        world.removeBody(this.chassisBody);
        world.removeEventListener("preStep", null) ; //this.preStepCallback);
        this.world = null;
    }

    public double castRay(WheelInfo wheel) {
        Vec3 rayvector = castRay_rayvector;
        Vec3 target = castRay_target;

        this.updateWheelTransformWorld(wheel);
        Body chassisBody = this.chassisBody;

        double depth = -1;

        double raylen = wheel.suspensionRestLength + wheel.radius;

        wheel.directionWorld.scale(raylen, rayvector);
        Vec3 source = wheel.chassisConnectionPointWorld;
        source.vadd(rayvector, target);
        RaycastResult raycastResult = wheel.raycastResult;

        double param = 0;

        raycastResult.reset();
        // Turn off ray collision with the chassis temporarily
        boolean oldState = chassisBody.collisionResponse;
        chassisBody.collisionResponse = false;

        // Cast ray against world
        this.world.rayTest(source, target, raycastResult);
        chassisBody.collisionResponse = oldState;

        Body object = raycastResult.body;

        wheel.raycastResult.groundObject = 0;

        if (object != null) {
            depth = raycastResult.distance;
            wheel.raycastResult.hitNormalWorld = raycastResult.hitNormalWorld;
            wheel.isInContact = true;

            double hitDistance = raycastResult.distance;
            wheel.suspensionLength = hitDistance - wheel.radius;

            // clamp on max suspension travel
            double minSuspensionLength = wheel.suspensionRestLength - wheel.maxSuspensionTravel ;
            double maxSuspensionLength = wheel.suspensionRestLength + wheel.maxSuspensionTravel ;
            if (wheel.suspensionLength < minSuspensionLength) {
                wheel.suspensionLength = minSuspensionLength;
            }
            if (wheel.suspensionLength > maxSuspensionLength) {
                wheel.suspensionLength = maxSuspensionLength;
                wheel.raycastResult.reset();
            }

            double denominator = wheel.raycastResult.hitNormalWorld.dot(wheel.directionWorld);

            Vec3 chassis_velocity_at_contactPoint = new Vec3();
            chassisBody.getVelocityAtWorldPoint(wheel.raycastResult.hitPointWorld, chassis_velocity_at_contactPoint);

            double projVel = wheel.raycastResult.hitNormalWorld.dot(chassis_velocity_at_contactPoint);

            if (denominator >= -0.1) {
                wheel.suspensionRelativeVelocity = 0;
                wheel.clippedInvContactDotSuspension = 1 / 0.1;
            } else {
                double inv = -1 / denominator;
                wheel.suspensionRelativeVelocity = projVel * inv;
                wheel.clippedInvContactDotSuspension = inv;
            }
        } else {
            // put wheel info as in rest position
            wheel.suspensionLength = wheel.suspensionRestLength + 0 * wheel.maxSuspensionTravel;
            wheel.suspensionRelativeVelocity = 0.0;
            wheel.directionWorld.scale(-1, wheel.raycastResult.hitNormalWorld);
            wheel.clippedInvContactDotSuspension = 1.0;
        }

        return depth;
    }

    public void updateWheelTransformWorld(WheelInfo wheel) {
        wheel.isInContact = false;
        Body chassisBody = this.chassisBody;
        chassisBody.pointToWorldFrame(wheel.chassisConnectionPointLocal, wheel.chassisConnectionPointWorld);
        chassisBody.vectorToWorldFrame(wheel.directionLocal, wheel.directionWorld);
        chassisBody.vectorToWorldFrame(wheel.axleLocal, wheel.axleWorld);
    }

    /**
     * Update one of the wheel transforms.
     * Note when rendering wheels: during each step, wheel transforms are updated
     * BEFORE the chassis; ie. their position becomes invalid after the step. Thus
     * when you render wheels, you must update wheel transforms before rendering
     * them. See raycastVehicle demo for an example.
     * 
     * @param wheelIndex The wheel index to update.
     */
    public void updateWheelTransform(int wheelIndex) {
        Vec3 up = tmpVec4;
        Vec3 right = tmpVec5;
        Vec3 fwd = tmpVec6;

        WheelInfo wheel = this.wheelInfos.get(wheelIndex);
        updateWheelTransformWorld(wheel);

        wheel.directionLocal.scale(-1, up);
        right.copy(wheel.axleLocal);
        up.cross(right, fwd);
        fwd.normalize();
        right.normalize();

        // Rotate around steering over the wheelAxle
        double steering = wheel.steering;
        Quaternion steeringOrn = new Quaternion();
        steeringOrn.setFromAxisAngle(up, steering);

        Quaternion rotatingOrn = new Quaternion();
        rotatingOrn.setFromAxisAngle(right, wheel.rotation);

        // World rotation of the wheel
        Quaternion q = wheel.worldTransform.quaternion;
        this.chassisBody.quaternion.mult(steeringOrn, q);
        q.mult(rotatingOrn, q);

        q.normalize();

        // world position of the wheel
        Vec3 p = wheel.worldTransform.position;
        p.copy(wheel.directionWorld);
        p.scale(wheel.suspensionLength, p);
        p.vadd(wheel.chassisConnectionPointWorld, p);
    }

    /**
     * Get the world transform of one of the wheels
     */
    public Transform getWheelTransformWorld(int wheelIndex) {
        return this.wheelInfos.get(wheelIndex).worldTransform;
    }

    public void updateFriction(double timeStep) {
        Vec3 surfNormalWS_scaled_proj = updateFriction_surfNormalWS_scaled_proj;
    
        // Calculate the impulse, so that the wheels don't move sidewards
        List<WheelInfo> wheelInfos = this.wheelInfos;
        int numWheels = wheelInfos.size();
        Body chassisBody = this.chassisBody;
        Vec3[] forwardWS = new Vec3[numWheels];
        Vec3[] axle = new Vec3[numWheels];
    
        this.numWheelsOnGround = 0;
    
        for (int i = 0; i < numWheels; i++) {
            WheelInfo wheel = wheelInfos.get(i);
            Body groundObject = wheel.raycastResult.body;
    
            if (groundObject != null) {
                this.numWheelsOnGround++;
            }
    
            wheel.sideImpulse = 0;
            wheel.forwardImpulse = 0;
            if (forwardWS[i] == null) {
                forwardWS[i] = new Vec3();
            }
            if (axle[i] == null) {
                axle[i] = new Vec3();
            }
        }
    
        for (int i = 0; i < numWheels; i++) {
            WheelInfo wheel = wheelInfos.get(i);
            Body groundObject = wheel.raycastResult.body;
    
            if (groundObject != null) {
                Vec3 axlei = axle[i];
                Transform wheelTrans = getWheelTransformWorld(i);
    
                // Get world axle
                wheelTrans.vectorToWorldFrame(directions[this.indexRightAxis], axlei);
    
                Vec3 surfNormalWS = wheel.raycastResult.hitNormalWorld;
                double proj = axlei.dot(surfNormalWS);
                surfNormalWS.scale(proj, surfNormalWS_scaled_proj);
                axlei.vsub(surfNormalWS_scaled_proj, axlei);
                axlei.normalize();
    
                surfNormalWS.cross(axlei, forwardWS[i]);
                forwardWS[i].normalize();
    
                wheel.sideImpulse = resolveSingleBilateral(
                        chassisBody,
                        wheel.raycastResult.hitPointWorld,
                        groundObject,
                        wheel.raycastResult.hitPointWorld,
                        axlei
                );
    
                wheel.sideImpulse *= sideFrictionStiffness2;
            }
        }
    
        double sideFactor = 1;
        double fwdFactor = 0.5;
    
        this.sliding = false;
        for (int i = 0; i < numWheels; i++) {
            WheelInfo wheel = wheelInfos.get(i);
            Body groundObject = wheel.raycastResult.body;
    
            double rollingFriction = 0;
    
            wheel.slipInfo = 1;
            if (groundObject != null) {
                double defaultRollingFrictionImpulse = 0;
                double maxImpulse = wheel.brake != 0 ? wheel.brake : defaultRollingFrictionImpulse;
    
                rollingFriction = calcRollingFriction(
                        chassisBody,
                        groundObject,
                        wheel.raycastResult.hitPointWorld,
                        forwardWS[i],
                        maxImpulse
                );
    
                rollingFriction += wheel.engineForce * timeStep;
    
                double factor = maxImpulse / rollingFriction;
                wheel.slipInfo *= factor;
            }
    
            wheel.forwardImpulse = 0;
            wheel.skidInfo = 1; 
    
            if (groundObject != null) {
                wheel.skidInfo = 1;
    
                double maximp = wheel.suspensionForce * timeStep * wheel.frictionSlip;
                double maximpSide = maximp;
    
                double maximpSquared = maximp * maximpSide;
    
                wheel.forwardImpulse = rollingFriction;
    
                double x = (wheel.forwardImpulse * fwdFactor) / wheel.forwardAcceleration;
                double y = (wheel.sideImpulse * sideFactor) / wheel.sideAcceleration;
    
                double impulseSquared = x * x + y * y;
    
                wheel.sliding = false;
                if (impulseSquared > maximpSquared) {
                    this.sliding = true;
                    wheel.sliding = true;
    
                    double factor = maximp / Math.sqrt(impulseSquared);
    
                    wheel.skidInfo *= factor;
                }
            }
        }
    
        if (this.sliding) {
            for (int i = 0; i < numWheels; i++) {
                WheelInfo wheel = wheelInfos.get(i);
                if (wheel.sideImpulse != 0) {
                    if (wheel.skidInfo < 1) {
                        wheel.forwardImpulse *= wheel.skidInfo;
                        wheel.sideImpulse *= wheel.skidInfo;
                    }
                }
            }
        }
    
        // Apply the impulses
        for (int i = 0; i < numWheels; i++) {
            WheelInfo wheel = wheelInfos.get(i);
    
            Vec3 rel_pos = new Vec3();
            wheel.raycastResult.hitPointWorld.vsub(chassisBody.position, rel_pos);
    
            if (wheel.forwardImpulse != 0) {
                Vec3 impulse = new Vec3();
                forwardWS[i].scale(wheel.forwardImpulse, impulse);
                chassisBody.applyImpulse(impulse, rel_pos);
            }
    
            if (wheel.sideImpulse != 0) {
                Body groundObject = wheel.raycastResult.body;
    
                Vec3 rel_pos2 = new Vec3();
                wheel.raycastResult.hitPointWorld.vsub(groundObject.position, rel_pos2);
                Vec3 sideImp = new Vec3();
                axle[i].scale(wheel.sideImpulse, sideImp);
    
                // Scale the relative position in the up direction with rollInfluence.
                // If rollInfluence is 1, the impulse will be applied on the hitPoint (easy to roll over), if it is zero it will be applied in the same plane as the center of mass (not easy to roll over).
                chassisBody.vectorToLocalFrame(rel_pos, rel_pos);
                if(this.indexUpAxis == 0) {
                	rel_pos.x *= wheel.rollInfluence ;
                }
                else if(this.indexUpAxis == 1) {
                	rel_pos.y *= wheel.rollInfluence ;
                }
                else {
                	rel_pos.z *= wheel.rollInfluence ;
                }
                
                //rel_pos.set("xyz".charAt(this.indexUpAxis), rel_pos.get("xyz".charAt(this.indexUpAxis)) * wheel.rollInfluence);
                
                chassisBody.vectorToWorldFrame(rel_pos, rel_pos);
                chassisBody.applyImpulse(sideImp, rel_pos);
    
                // Apply friction impulse on the ground
                sideImp.scale(-1, sideImp);
                groundObject.applyImpulse(sideImp, rel_pos2);
            }
        }
    }
    

    private Vec3 tmpVec1 = new Vec3();
    private Vec3 tmpVec2 = new Vec3();
    private Vec3 tmpVec3 = new Vec3();
    private Vec3 tmpVec4 = new Vec3();
    private Vec3 tmpVec5 = new Vec3();
    private Vec3 tmpVec6 = new Vec3();
    private Ray tmpRay = new Ray();

    private Vec3 torque = new Vec3();

    private Vec3 castRay_rayvector = new Vec3();
    private Vec3 castRay_target = new Vec3();

    private Vec3[] directions = {
        new Vec3(1, 0, 0),
        new Vec3(0, 1, 0),
        new Vec3(0, 0, 1)
    };

    private Vec3 updateFriction_surfNormalWS_scaled_proj = new Vec3();
    private List<Vec3> updateFriction_axle = new ArrayList<Vec3>() ; 
    private List<Vec3> updateFriction_forwardWS = new ArrayList<>() ; 
    private double sideFrictionStiffness2 = 1;

    private  Vec3 calcRollingFriction_vel1 = new Vec3();
    private  Vec3 calcRollingFriction_vel2 = new Vec3();
    private Vec3 calcRollingFriction_vel = new Vec3();

    /**
     * Calculate rolling friction between two bodies.
     *
     * @param body0                The first body.
     * @param body1                The second body.
     * @param frictionPosWorld     The position of the friction.
     * @param frictionDirectionWorld The direction of the friction.
     * @param maxImpulse           The maximum impulse.
     * @return The rolling friction value.
     */
    private double calcRollingFriction(
            Body body0,
            Body body1,
            Vec3 frictionPosWorld,
            Vec3 frictionDirectionWorld,
            double maxImpulse
    ) {
        double j1 = 0;
        Vec3 contactPosWorld = frictionPosWorld;

        Vec3 vel1 = calcRollingFriction_vel1;
        Vec3 vel2 = calcRollingFriction_vel2;
        Vec3 vel = calcRollingFriction_vel;

        body0.getVelocityAtWorldPoint(contactPosWorld, vel1);
        body1.getVelocityAtWorldPoint(contactPosWorld, vel2);
        vel1.vsub(vel2, vel);

        double vrel = frictionDirectionWorld.dot(vel);

        double denom0 = computeImpulseDenominator(body0, frictionPosWorld, frictionDirectionWorld);
        double denom1 = computeImpulseDenominator(body1, frictionPosWorld, frictionDirectionWorld);
        double relaxation = 1;
        double jacDiagABInv = relaxation / (denom0 + denom1);

        j1 = -vrel * jacDiagABInv;

        if (maxImpulse < j1) {
            j1 = maxImpulse;
        }
        if (j1 < -maxImpulse) {
            j1 = -maxImpulse;
        }

        return j1;
    }

    private Vec3 computeImpulseDenominator_r0 = new Vec3();
    private Vec3 computeImpulseDenominator_c0 = new Vec3();
    private Vec3 computeImpulseDenominator_vec = new Vec3();
    private Vec3 computeImpulseDenominator_m = new Vec3();

    /**
     * Compute the impulse denominator.
     *
     * @param body   The body.
     * @param pos    The position.
     * @param normal The normal vector.
     * @return The impulse denominator.
     */
    private double computeImpulseDenominator(Body body, Vec3 pos, Vec3 normal) {
        Vec3 r0 = computeImpulseDenominator_r0;
        Vec3 c0 = computeImpulseDenominator_c0;
        Vec3 vec = computeImpulseDenominator_vec;
        Vec3 m = computeImpulseDenominator_m;

        pos.vsub(body.position, r0);
        r0.cross(normal, c0);
        body.invInertiaWorld.vmult(c0, m);
        m.cross(r0, vec);

        return body.invMass + normal.dot(vec);
    }

    private Vec3 resolveSingleBilateral_vel1 = new Vec3();
    private Vec3 resolveSingleBilateral_vel2 = new Vec3();
    private Vec3 resolveSingleBilateral_vel = new Vec3();

    /**
     * Resolve a single bilateral constraint between two bodies.
     *
     * @param body1  The first body.
     * @param pos1   The position on the first body.
     * @param body2  The second body.
     * @param pos2   The position on the second body.
     * @param normal The normal vector.
     * @return The impulse value.
     */
    private double resolveSingleBilateral(Body body1, Vec3 pos1, Body body2, Vec3 pos2, Vec3 normal) {
        double normalLenSqr = normal.lengthSquared();
        if (normalLenSqr > 1.1) {
            return 0;
        }

        Vec3 vel1 = resolveSingleBilateral_vel1;
        Vec3 vel2 = resolveSingleBilateral_vel2;
        Vec3 vel = resolveSingleBilateral_vel;

        body1.getVelocityAtWorldPoint(pos1, vel1);
        body2.getVelocityAtWorldPoint(pos2, vel2);

        vel1.vsub(vel2, vel);

        double rel_vel = normal.dot(vel);

        double contactDamping = 0.2;
        double massTerm = 1 / (body1.invMass + body2.invMass);
        double impulse = -contactDamping * rel_vel * massTerm;

        return impulse;
    }
}


