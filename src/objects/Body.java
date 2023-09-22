package objects;

import java.util.ArrayList;

import math.Vec3;
import math.JacobianElement;
import math.Mat3;
import objects.Body;
import shapes.Box;
import shapes.Shape;
import material.Material;
import world.World;
import math.Quaternion;
import collision.AABB;
import utils.EventTarget;

//public class Body extends EventTarget{
//	BodyTypes bodyType= BodyTypes.DYNAMIC;

//}

import java.util.List;

import javax.swing.text.Position;

import org.w3c.dom.events.Event;

/**
 * Base class for all body types.
 * 
 * @example
 *          const shape = new CANNON.Sphere(1)
 *          const body = new CANNON.Body({
 *          mass: 1,
 *          shape,
 *          })
 *          world.addBody(body)
 */

public class Body extends EventTarget {
    static int idCounter = 0;

    /**
     * Dispatched after two bodies collide. This event is dispatched on each
     * of the two bodies involved in the collision.
     * 
     * @event collide
     * @param body    The body that was involved in the collision.
     * @param contact The details of the collision.
     */
    public static final String COLLIDE_EVENT_NAME = "collide";
    /**
     * A dynamic body is fully simulated. Can be moved manually by the user, but
     * normally they move according to forces. A dynamic body can collide with all
     * body types. A dynamic body always has finite, non-zero mass.
     */
    public static final int DYNAMIC = BodyTypes.DYNAMIC.getValue();
    /**
     * A static body does not move during simulation and behaves as if it has
     * infinite mass. Static bodies can be moved manually by setting the position of
     * the body. The velocity of a static body is always zero. Static bodies do not
     * collide with other static or kinematic bodies.
     */
    public static final int STATIC = BodyTypes.STATIC.getValue();
    /**
     * A kinematic body moves under simulation according to its velocity. They do
     * not respond to forces. They can be moved manually, but normally a kinematic
     * body is moved by setting its velocity. A kinematic body behaves as if it has
     * infinite mass. Kinematic bodies do not collide with other static or kinematic
     * bodies.
     */
    public static final int KINEMATIC = BodyTypes.KINEMATIC.getValue();
    public static final int AWAKE = BodySleepStates.AWAKE.getValue();
    public static final int SLEEPY = BodySleepStates.SLEEPY.getValue();
    public static final int SLEEPING = BodySleepStates.SLEEPING.getValue();
    // changed , wasn't initially in the source code , COLLIDE_EVENT
    // public static final Event COLLIDE_EVENT = new Event(COLLIDE_EVENT_NAME);

    /**
     * Dispatched after a sleeping body has woken up.
     * 
     * @event wakeup
     */
    public static final Event WAKEUP_EVENT = new Event("wakeup");
    /**
     * Dispatched after a sleeping body has woken up.
     * 
     * @event wakeup
     */
    public static final Event SLEEPY_EVENT = new Event("sleepy");
    /**
     * Dispatched after a body has fallen asleep.
     * 
     * @event sleep
     */
    public static final Event SLEEP_EVENT = new Event("sleep");

    public int id;// Identifier of the body.
    public int index; // Position of body in World.bodies. Updated by World and used in
                      // ArrayCollisionMatrix.
    public World world;// Reference to the world the body is living in.
    public Vec3 vlambda;
    /**
     * The collision group the body belongs to.
     * 
     * @default 1
     */
    public int collisionFilterGroup;
    /**
     * The collision group the body can collide with.
     * 
     * @default -1
     */
    public int collisionFilterMask;
    /**
     * Whether to produce contact forces when in contact with other bodies. Note
     * that contacts will be generated, but they will be disabled - i.e. "collide"
     * events will be raised, but forces will not be altered.
     */
    public boolean collisionResponse;
    /**
     * World space position of the body.
     */
    public Vec3 position;
    public Vec3 previousPosition;
    /**
     * Interpolated position of the body.
     */
    public Vec3 interpolatedPosition;
    /**
     * Initial position of the body.
     */
    public Vec3 initPosition;
    /**
     * World space velocity of the body.
     */
    public Vec3 velocity;
    /**
     * Initial velocity of the body.
     */
    public Vec3 initVelocity;
    /**
     * Linear force on the body in world space.
     */
    public Vec3 force;
    /**
     * The mass of the body.
     * 
     * @default 0
     */
    public double mass;
    public double invMass;
    /**
     * The physics material of the body. It defines the body interaction with other
     * bodies.
     */
    public Material material;
    /**
     * How much to damp the body velocity each step. It can go from 0 to 1.
     * 
     * @default 0.01
     */
    public double linearDamping;
    /**
     * One of: `Body.DYNAMIC`, `Body.STATIC` and `Body.KINEMATIC`.
     */
    public int type;
    /**
     * If true, the body will automatically fall to sleep.
     * 
     * @default true
     */
    public boolean allowSleep;
    public int sleepState;// current sleep state
    /**
     * If the speed (the norm of the velocity) is smaller than this value, the body
     * is considered sleepy.
     * 
     * @default 0.1
     */
    public double sleepSpeedLimit;
    /**
     * If the body has been sleepy for this sleepTimeLimit seconds, it is considered
     * sleeping.
     * 
     * @default 1
     */
    public double sleepTimeLimit;
    public double timeLastSleepy;
    public boolean wakeUpAfterNarrowphase;
    /**
     * World space rotational force on the body, around center of mass.
     */
    public Vec3 torque;
    /**
     * World space orientation of the body.
     */
    public Quaternion quaternion;
    /**
     * Initial quaternion of the body.
     */
    public Quaternion initQuaternion;
    public Quaternion previousQuaternion;
    /**
     * Interpolated orientation of the body.
     */
    public Quaternion interpolatedQuaternion;
    /**
     * Angular velocity of the body, in world space. Think of the angular velocity
     * as a vector, which the body rotates around. The length of this vector
     * determines how fast (in radians per second) the body rotates.
     */
    public Vec3 angularVelocity;
    /**
     * Initial angular velocity of the body.
     */
    public Vec3 initAngularVelocity;
    /**
     * List of Shapes that have been added to the body.
     */
    public List<Shape> shapes;
    /**
     * Position of each Shape in the body, given in local Body space.
     */
    public List<Vec3> shapeOffsets;
    /**
     * Orientation of each Shape, given in local Body space.
     */
    public List<Quaternion> shapeOrientations;
    /**
     * The inertia of the body.
     */
    Vec3 inertia;
    Vec3 invInertia;
    Mat3 invInertiaWorld;
    public double invMassSolve;
    Vec3 invInertiaSolve;
    public Mat3 invInertiaWorldSolve;
    /**
     * Set to true if you don't want the body to rotate. Make sure to run
     * .updateMassProperties() if you change this after the body creation.
     * 
     * @default false
     */
    boolean fixedRotation;
    /**
     * How much to damp the body angular velocity each step. It can go from 0 to 1.
     * 
     * @default 0.01
     */
    public double angularDamping;
    /**
     * Use this property to limit the motion along any world axis. (1,1,1) will
     * allow motion along all axes while (0,0,0) allows none.
     */
    public Vec3 linearFactor;
    /**
     * Use this property to limit the rotational motion along any world axis.
     * (1,1,1) will allow rotation along all axes while (0,0,0) allows none.
     */
    public Vec3 angularFactor;
    /**
     * World space bounding box of the body and its shapes.
     */
    public AABB aabb;
    /**
     * Indicates if the AABB needs to be updated before use.
     */
    public boolean aabbNeedsUpdate;
    /**
     * Total bounding radius of the Body including its shapes, relative to
     * body.position.
     */
    public double boundingRadius;
    public Vec3 wlambda;
    /**
     * When true the body behaves like a trigger. It does not collide
     * with other bodies but collision events are still triggered.
     * 
     * @default false
     */
    public boolean isTrigger;

    public Body() {
        this(new BodyOptions());
    }

    public Body(BodyOptions options) {
        super();
        this.id = idCounter++;
        this.index = -1;
        this.world = null;
        this.vlambda = new Vec3();
        this.collisionFilterGroup = options.collisionFilterGroup;
        this.collisionFilterMask = options.collisionFilterMask;
        this.collisionResponse = options.collisionResponse;
        this.position = new Vec3();
        this.previousPosition = new Vec3();
        this.interpolatedPosition = new Vec3();
        this.initPosition = new Vec3();

        if (options.position != null) {
            this.position.copy(options.position);
            this.previousPosition.copy(options.position);
            this.interpolatedPosition.copy(options.position);
            this.initPosition.copy(options.position);
        }

        this.velocity = new Vec3();

        if (options.velocity != null) {
            this.velocity.copy(options.velocity);
        }

        this.initVelocity = new Vec3();
        this.force = new Vec3();
        // added , removed the needless code of checking for mass == null
        double mass = options.mass; // options.mass != null ? options.mass : 0;
        this.mass = mass;
        this.invMass = mass > 0 ? 1.0 / mass : 0;
        this.material = options.material;
        this.linearDamping = 0.01; // options.linearDamping != null ? options.linearDamping : 0.01;

        this.type = mass <= 0.0 ? STATIC : DYNAMIC;

        if (options.type != null) {
            this.type = options.type.getValue();
        }

        this.allowSleep = options.allowSleep; // options.allowSleep != null ? options.allowSleep : true;
        this.sleepState = AWAKE;
        // removed all checkers
        this.sleepSpeedLimit = options.sleepSpeedLimit; // options.sleepSpeedLimit != null ? options.sleepSpeedLimit :
                                                        // 0.1;
        this.sleepTimeLimit = options.sleepTimeLimit; // options.sleepTimeLimit != null ? options.sleepTimeLimit : 1;
        this.timeLastSleepy = 0;
        this.wakeUpAfterNarrowphase = false;

        this.torque = new Vec3();
        this.quaternion = new Quaternion();
        this.initQuaternion = new Quaternion();
        this.previousQuaternion = new Quaternion();
        this.interpolatedQuaternion = new Quaternion();

        if (options.quaternion != null) {
            this.quaternion.copy(options.quaternion);
            this.initQuaternion.copy(options.quaternion);
            this.previousQuaternion.copy(options.quaternion);
            this.interpolatedQuaternion.copy(options.quaternion);
        }

        this.angularVelocity = new Vec3();

        if (options.angularVelocity != null) {
            this.angularVelocity.copy(options.angularVelocity);
        }

        this.initAngularVelocity = new Vec3();
        this.shapes = new ArrayList<>();
        this.shapeOffsets = new ArrayList<>();
        this.shapeOrientations = new ArrayList<>();
        this.inertia = new Vec3();
        this.invInertia = new Vec3();
        this.invInertiaWorld = new Mat3();
        this.invMassSolve = 0;
        this.invInertiaSolve = new Vec3();
        this.invInertiaWorldSolve = new Mat3();
        this.fixedRotation = options.fixedRotation; // options.fixedRotation != null ? options.fixedRotation : false;
        this.angularDamping = options.angularDamping; // options.angularDamping != null ? options.angularDamping : 0.01;
        this.linearFactor = new Vec3(1, 1, 1);

        if (options.linearFactor != null) {
            this.linearFactor.copy(options.linearFactor);
        }

        this.angularFactor = new Vec3(1, 1, 1);

        if (options.angularFactor != null) {
            this.angularFactor.copy(options.angularFactor);
        }

        this.aabb = new AABB();
        this.aabbNeedsUpdate = true;
        this.boundingRadius = 0;
        this.wlambda = new Vec3();
        this.isTrigger = options.isTrigger; // options.isTrigger != null ? options.isTrigger : false;

        if (options.shape != null) {
            this.addShape(options.shape);
        }

        this.updateMassProperties();
    }

    /**
     * Wake the body up.
     */
    public void wakeUp() {
        int prevState = sleepState;
        sleepState = AWAKE;
        wakeUpAfterNarrowphase = false;
        if (prevState == SLEEPING) {
            dispatchEvent(wakeupEvent);
        }
    }

    /**
     * Force body sleep
     */
    public void sleep() {
        sleepState = SLEEPING;
        velocity.set(0, 0, 0);
        angularVelocity.set(0, 0, 0);
        wakeUpAfterNarrowphase = false;
    }

    /**
     * Called every timestep to update internal sleep timer and change sleep state
     * if needed.
     * 
     * @param time The world time in seconds
     */
    public void sleepTick(double time) {
        if (allowSleep) {
            int sleepState = this.sleepState;
            double speedSquared = velocity.lengthSquared() + angularVelocity.lengthSquared();
            double speedLimitSquared = sleepSpeedLimit * sleepSpeedLimit;

            if (sleepState == AWAKE && speedSquared < speedLimitSquared) {
                this.sleepState = SLEEPY;
                timeLastSleepy = time;
                dispatchEvent(sleepyEvent);
            } else if (sleepState == SLEEPY && speedSquared > speedLimitSquared) {
                wakeUp();
            } else if (sleepState == SLEEPY && time - timeLastSleepy > sleepTimeLimit) {
                sleep();
                dispatchEvent(sleepEvent);
            }
        }
    }

    /**
     * If the body is sleeping, it should be immovable / have infinite mass during
     * solve. We solve it by having a separate "solve mass".
     */
    public void updateSolveMassProperties() {
        if (sleepState == SLEEPING || type == Body.KINEMATIC) {
            invMassSolve = 0;
            invInertiaSolve.setZero();
            invInertiaWorldSolve.setZero();
        } else {
            invMassSolve = invMass;
            invInertiaSolve.copy(invInertia);
            invInertiaWorldSolve.copy(invInertiaWorld);
        }
    }

    /**
     * Convert a world point to local body frame.
     */
    public Vec3 pointToLocalFrame(Vec3 worldPoint, Vec3 result) {
        worldPoint.vsub(position, result);
        quaternion.conjugate().vmult(result, result);
        return result;
    }

    // added
    public Vec3 vectorToLocalFrame(Vec3 worldVector) {
        return vectorToLocalFrame(worldVector, new Vec3());
    }

    /**
     * Convert a world vector to local body frame.
     */
    public Vec3 vectorToLocalFrame(Vec3 worldVector, Vec3 result) {
        quaternion.conjugate().vmult(worldVector, result);
        return result;
    }

    /**
     * Convert a local body point to world frame.
     */
    public Vec3 pointToWorldFrame(Vec3 localPoint, Vec3 result) {
        quaternion.vmult(localPoint, result);
        result.vadd(position, result);
        return result;
    }

    /**
     * Convert a local body point to world frame.
     */
    public Vec3 vectorToWorldFrame(Vec3 localVector, Vec3 result) {
        quaternion.vmult(localVector, result);
        return result;
    }

    public Body addShape(Shape shape) {
        addShape(shape, new Vec3(), new Quaternion());
    }

    /**
     * Add a shape to the body with a local offset and orientation.
     * 
     * @return The body object, for chainability.
     */
    public Body addShape(Shape shape, Vec3 offset, Quaternion orientation) {
        Vec3 offsetCopy = offset;
        Quaternion orientationCopy = orientation;

        shapes.add(shape);
        shapeOffsets.add(offsetCopy);
        shapeOrientations.add(orientationCopy);
        updateMassProperties();
        updateBoundingRadius();
        aabbNeedsUpdate = true;
        shape.body = this;

        return this;
    }

    /**
     * Remove a shape from the body.
     * 
     * @return The body object, for chainability.
     */
    public Body removeShape(Shape shape) {
        int index = shapes.indexOf(shape);

        if (index != -1) {
            shapes.remove(index);
            shapeOffsets.remove(index);
            shapeOrientations.remove(index);
            updateMassProperties();
            updateBoundingRadius();
            aabbNeedsUpdate = true;
            shape.body = null;
        } else {
            System.out.println("Shape does not belong to the body");
        }

        return this;
    }

    /**
     * Update the bounding radius of the body. Should be done if any of the shapes
     * are changed.
     */
    public void updateBoundingRadius() {
        double radius = 0;

        for (int i = 0; i < shapes.size(); i++) {
            Shape shape = shapes.get(i);
            shape.updateBoundingSphereRadius();
            double offset = shapeOffsets.get(i).length();
            double r = shape.boundingSphereRadius;

            if (offset + r > radius) {
                radius = offset + r;
            }
        }

        boundingRadius = radius;
    }

    /**
     * Updates the .aabb
     */
    public void updateAABB() {
        int N = shapes.size();
        Vec3 offset = new Vec3();
        Quaternion orientation = new Quaternion();
        Quaternion bodyQuat = quaternion;
        AABB shapeAABB = new AABB();

        for (int i = 0; i < N; i++) {
            Shape shape = shapes.get(i);

            // Get shape world position
            bodyQuat.vmult(shapeOffsets.get(i), offset);
            offset.vadd(position, offset);

            // Get shape world quaternion
            bodyQuat.mult(shapeOrientations.get(i), orientation);
            shape.calculateWorldAABB(offset, orientation, shapeAABB.lowerBound, shapeAABB.upperBound);

            if (i == 0) {
                aabb.copy(shapeAABB);
            } else {
                aabb.extend(shapeAABB);
            }
        }

        aabbNeedsUpdate = false;
    }

    public void updateInertiaWorld() {
        updateInertiaWorld(false);
    }

    /**
     * Update `.inertiaWorld` and `.invInertiaWorld`
     */
    public void updateInertiaWorld(boolean force) {
        Vec3 I = invInertia;

        if (I.x == I.y && I.y == I.z && !force) {
            // If inertia M = s*I, where I is identity and s a scalar, then
            // R*M*R' = R*(s*I)*R' = s*R*I*R' = s*R*R' = s*I = M
            // where R is the rotation matrix.
            // In other words, we don't have to transform the inertia if all
            // inertia diagonal entries are equal.
        } else {
            Mat3 m1 = new Mat3();
            Mat3 m2 = new Mat3();
            Mat3 m3 = new Mat3();
            m1.setRotationFromQuaternion(quaternion);
            m1.transpose(m2);
            m1.scale(I, m1);
            m1.mmult(m2, invInertiaWorld);
        }
    }

    /**
     * Apply force to a point of the body. This could for example be a point on the
     * Body surface.
     * Applying force this way will add to Body.force and Body.torque.
     * 
     * @param force         The amount of force to add.
     * @param relativePoint A point relative to the center of mass to apply the
     *                      force on.
     */
    public void applyForce(Vec3 force, Vec3 relativePoint) {
        if (type != DYNAMIC) {
            return;
        }

        if (sleepState == SLEEPING) {
            wakeUp();
        }
        // Compute produced rotational force
        Vec3 rotForce = new Vec3();
        relativePoint.cross(force, rotForce);
        // Add linear force
        // added this.force to store the result of force added
        this.force.vadd(force, this.force);
        this.torque.vadd(rotForce, this.torque);
    }

    /**
     * Apply force to a local point in the body.
     * 
     * @param force      The force vector to apply, defined locally in the body
     *                   frame.
     * @param localPoint A local point in the body to apply the force on.
     */
    public void applyLocalForce(Vec3 localForce, Vec3 localPoint) {
        if (type != DYNAMIC) {
            return;
        }

        Vec3 worldForce = new Vec3();
        Vec3 relativePointWorld = new Vec3();

        vectorToWorldFrame(localForce, worldForce);
        vectorToWorldFrame(localPoint, relativePointWorld);

        applyForce(worldForce, relativePointWorld);
    }

    /**
     * Apply torque to the body.
     * 
     * @param torque The amount of torque to add.
     */
    public void applyTorque(Vec3 torque) {
        if (type != DYNAMIC) {
            return;
        }

        if (sleepState == SLEEPING) {
            wakeUp();
        }
        // Add rotational force
        this.torque.vadd(torque);
    }

    /**
     * Apply impulse to a point of the body. This could for example be a point on
     * the Body surface.
     * An impulse is a force added to a body during a short period of time (impulse
     * = force * time).
     * Impulses will be added to Body.velocity and Body.angularVelocity.
     * 
     * @param impulse       The amount of impulse to add.
     * @param relativePoint A point relative to the center of mass to apply the
     *                      force on.
     */
    public void applyImpulse(Vec3 impulse, Vec3 relativePoint) {
        if (type != DYNAMIC) {
            return;
        }

        if (sleepState == SLEEPING) {
            wakeUp();
        }
        // Compute point position relative to the body center
        Vec3 r = relativePoint;

        // Compute produced central impulse velocity
        Vec3 velo = new Vec3();
        velo.copy(impulse);
        velo.scale(this.invMass, velo);

        // Add linear impulse
        this.velocity.vadd(velo,this.velocity);

        // Compute produced rotational impulse velocity
        Vec3 rotVelo = new Vec3();
        r.cross(impulse, rotVelo);

        /*
         * rotVelo.x *= this.invInertia.x;
         * rotVelo.y *= this.invInertia.y;
         * rotVelo.z *= this.invInertia.z;
         */
        this.invInertiaWorld.vmult(rotVelo, rotVelo);

        angularVelocity.vadd(rotVelo , this.angularVelocity);
    }

    /**
   * Apply locally-defined impulse to a local point in the body.
   * @param force The force vector to apply, defined locally in the body frame.
   * @param localPoint A local point in the body to apply the force on.
   */
    public void applyLocalImpulse(Vec3 localImpulse, Vec3 localPoint) {
        if (type != DYNAMIC) {
            return;
        }

        Vec3 worldImpulse = new Vec3();
        Vec3 relativePointWorld = new Vec3();

        vectorToWorldFrame(localImpulse, worldImpulse);
        vectorToWorldFrame(localPoint, relativePointWorld);

        applyImpulse(worldImpulse, relativePointWorld);
    }

    /**
   * Should be called whenever you change the body shape or mass.
   */
    public void updateMassProperties() {
        Vec3 halfExtents = new Vec3();

        invMass = (mass > 0) ? 1.0 / mass : 0;
        Vec3 I = this.inertia;
        boolean fixed = fixedRotation;

        updateAABB();
        halfExtents.set(
                (this.aabb.upperBound.x - this.aabb.lowerBound.x) / 2,
                (this.aabb.upperBound.y - this.aabb.lowerBound.y) / 2,
                (this.aabb.upperBound.z - this.aabb.lowerBound.z) / 2);
        // halfExtents.vsub(aabb.upperBound, aabb.lowerBound);
        // halfExtents.scale(0.5, halfExtents);
        Box.calculateInertia(halfExtents, mass, I);

        this.invInertia.set(
                (I.x > 0 && !fixed) ? 1.0 / I.x : 0,
                (I.y > 0 && !fixed) ? 1.0 / I.y : 0,
                (I.z > 0 && !fixed) ? 1.0 / I.z : 0);
        updateInertiaWorld(true);
    }

    /**
   * Get world velocity of a point in the body.
   * @param worldPoint
   * @param result
   * @return The result vector.
   */
    public Vec3 getVelocityAtWorldPoint(Vec3 worldPoint, Vec3 result) {
        Vec3 r = new Vec3();
        worldPoint.vsub(position, r);
        r.cross(angularVelocity, result);
        result.vadd(velocity,result);
        return result;
    }

    /**
   * Move the body forward in time.
   * @param dt Time step
   * @param quatNormalize Set to true to normalize the body quaternion
   * @param quatNormalizeFast If the quaternion should be normalized using "fast" quaternion normalization
   */
    public void integrate(double dt, boolean quatNormalize, boolean quatNormalizeFast) {
        previousPosition.copy(position);
        previousQuaternion.copy(quaternion);

        if (!(type == DYNAMIC || type == KINEMATIC) || sleepState == SLEEPING) {
            return;
        }

        double iMdt = invMass * dt;
        velocity.x += force.x * iMdt * linearFactor.x;
        velocity.y += force.y * iMdt * linearFactor.y;
        velocity.z += force.z * iMdt * linearFactor.z;

        double[] e = this.invInertiaWorld.elements;// new double[9];
        // invInertiaWorld.get(e);
        double angularFactorX = angularFactor.x;
        double angularFactorY = angularFactor.y;
        double angularFactorZ = angularFactor.z;
        double tx = torque.x * angularFactorX;
        double ty = torque.y * angularFactorY;
        double tz = torque.z * angularFactorZ;
        angularVelocity.x += dt * (e[0] * tx + e[1] * ty + e[2] * tz);
        angularVelocity.y += dt * (e[3] * tx + e[4] * ty + e[5] * tz);
        angularVelocity.z += dt * (e[6] * tx + e[7] * ty + e[8] * tz);

        position.x += velocity.x * dt;
        position.y += velocity.y * dt;
        position.z += velocity.z * dt;

        quaternion.integrate(angularVelocity, dt, angularFactor, quaternion);

        if (quatNormalize) {
            if (quatNormalizeFast) {
                quaternion.normalizeFast();
            } else {
                quaternion.normalize();
            }
        }

        aabbNeedsUpdate = true;
        updateInertiaWorld();
    }

}
