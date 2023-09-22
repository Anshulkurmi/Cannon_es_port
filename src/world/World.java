package world;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.w3c.dom.events.Event;

import objects.Body;
import shapes.Shape;
import solver.GSSolver;
import solver.Solver;
import utils.EventTarget;
import utils.TupleDictionary;
import equations.ContactEquation;
import equations.Equation;
import equations.FrictionEquation;
import material.ContactMaterial;
import material.ContactMaterialOptions;
import material.Material;
import math.Vec3;
import collision.AABB;
import collision.ArrayCollisionMatrix;
import collision.Broadphase;
import collision.NaiveBroadphase;
import collision.OverlapKeeper;
import collision.Ray;
import collision.RayModes;
import collision.RayOptions;
import collision.RaycastCallback;
import collision.RaycastResult;
import constraints.Constraint;

/**
 * The physics world
 */
public class World extends EventTarget {
    /**
   * Currently / last used timestep. Is set to -1 if not available. This value is updated before each internal step, which means that it is "fresh" inside event callbacks.
   */
    double dt;
    /**
   * Makes bodies go to sleep when they've been inactive.
   * @default false
   */
    private boolean allowSleep;
    /**
   * All the current contacts (instances of ContactEquation) in the world.
   */
    private List<ContactEquation> contacts;
    private List<FrictionEquation> frictionEquations;
    /**
   * How often to normalize quaternions. Set to 0 for every step, 1 for every second etc.. A larger value increases performance. If bodies tend to explode, set to a smaller value (zero to be sure nothing can go wrong).
   * @default 0
   */
    private int quatNormalizeSkip;
    /**
   * Set to true to use fast quaternion normalization. It is often enough accurate to use.
   * If bodies tend to explode, set to false.
   * @default false
   */
    private boolean quatNormalizeFast;
    /**
   * The wall-clock time since simulation start.
   */
    private double time;
    /**
   * Number of timesteps taken since start.
   */
    private int stepnumber;
    /**
   * Default and last timestep sizes.
   */
    private double default_dt;
    private int nextId;
    /**
   * The gravity of the world.
   */
    Vec3 gravity;
    /**
   * Gravity to use when approximating the friction max force (mu \* mass \* gravity).
   * If undefined, global gravity will be used.
   * Use to enable friction in a World with a null gravity vector (no gravity).
   */
    Vec3 frictionGravity;
    /**
   * The broadphase algorithm to use.
   * @default NaiveBroadphase
   */
    private Broadphase broadphase;
    /**
   * All bodies in this world
   */
    public List<Body> bodies;
    /**
   * True if any bodies are not sleeping, false if every body is sleeping.
   */
    private boolean hasActiveBodies;
    /**
   * The solver algorithm to use.
   * @default GSSolver
   */
    private Solver solver;
    private List<Constraint> constraints;
    private Narrowphase narrowphase;
    /**
   * collisionMatrix
   */
    private ArrayCollisionMatrix collisionMatrix;
    /**
   * CollisionMatrix from the previous step.
   */
    private ArrayCollisionMatrix collisionMatrixPrevious;
    /**
   * All added contactmaterials.
   */
    protected OverlapKeeper bodyOverlapKeeper;
    protected OverlapKeeper shapeOverlapKeeper;
    /**
   * All added contactmaterials.
   */
    private List<ContactMaterial> contactmaterials;
    /**
   * Used to look up a ContactMaterial given two instances of Material.
   */
    protected TupleDictionary contactMaterialTable;
    private Material defaultMaterial;
    ContactMaterial defaultContactMaterial;
    private boolean doProfiling;
    private Profile profile;
    /**
   * Time accumulator for interpolation.
   * @see https://gafferongames.com/game-physics/fix-your-timestep/
   */
    private double accumulator;
    private List<Object> subsystems;
    /**
   * Dispatched after a body has been added to the world.
   */
    protected AddBodyEvent addBodyEvent;
    /**
   * Dispatched after a body has been removed from the world.
   */
    private RemoveBodyEvent removeBodyEvent;
    private Map<Integer, Body> idToBodyMap;
    private double lastCallTime;

    public World(WorldOptions options) {
        super();
        this.dt = -1;
        this.allowSleep = options.allowSleep ; // options.allowSleep != null ? options.allowSleep : false;
        this.contacts = new ArrayList<>();
        this.frictionEquations = new ArrayList<>();
        this.quatNormalizeSkip = options.quatNormalizeSkip;//options.quatNormalizeSkip != null ? options.quatNormalizeSkip : 0;
        this.quatNormalizeFast = options.quatNormalizeFast ;//options.quatNormalizeFast != null ? options.quatNormalizeFast : false;
        this.time = 0.0;
        this.stepnumber = 0;
        this.default_dt = 1 / 60;
        this.nextId = 0;
        this.gravity = new Vec3();
        if (options.gravity != null) {
            this.gravity.copy(options.gravity);
        }
        if (options.frictionGravity != null) {
            this.frictionGravity = new Vec3();
            this.frictionGravity.copy(options.frictionGravity);
        }
        this.broadphase = options.broadphase != null ? options.broadphase : new NaiveBroadphase();
        this.bodies = new ArrayList<>();
        this.hasActiveBodies = false;
        this.solver = options.solver != null ? options.solver : new GSSolver();
        this.constraints = new ArrayList<>();
        this.narrowphase = new Narrowphase(this);
        this.collisionMatrix = new ArrayCollisionMatrix();
        this.collisionMatrixPrevious = new ArrayCollisionMatrix();
        this.bodyOverlapKeeper = new OverlapKeeper();
        this.shapeOverlapKeeper = new OverlapKeeper();
        this.contactmaterials = new ArrayList<>();
        this.contactMaterialTable = new TupleDictionary();
        this.defaultMaterial = new Material("default");
        this.defaultContactMaterial = new ContactMaterial(this.defaultMaterial, this.defaultMaterial, new ContactMaterialOptions(0.3, 0.0, 1e7,3,1e7,3));
        this.doProfiling = false;
        this.profile = new Profile();
        this.accumulator = 0;
        this.subsystems = new ArrayList<>();
        this.addBodyEvent = new AddBodyEvent("addBody", null);
        this.removeBodyEvent = new RemoveBodyEvent("removeBody", null);
        this.idToBodyMap = new HashMap<>();
        this.broadphase.setWorld(this);
    }

    /**
   * Get the contact material between materials m1 and m2
   * @return The contact material if it was found.
   */
    public ContactMaterial getContactMaterial(Material m1, Material m2) {
        // Assuming you have a contactMaterialTable similar to a Map in TypeScript
        //added : typecasted return type object as ContactMaterial
        return (ContactMaterial)contactMaterialTable.get(m1.id, m2.id);
    }

    /**
   * Store old collision state info
   */
    public void collisionMatrixTick() {
        ArrayCollisionMatrix temp = collisionMatrixPrevious;
        collisionMatrixPrevious = collisionMatrix;
        collisionMatrix = temp;
        collisionMatrix.reset();
        
        bodyOverlapKeeper.tick();
        shapeOverlapKeeper.tick();
    }
    
    /**
   * Add a constraint to the simulation.
   */
    public void addConstraint(Constraint c) {
        constraints.add(c);
    }

    /**
   * Removes a constraint
   */
    public void removeConstraint(Constraint c) {
        int idx = constraints.indexOf(c);
        if (idx != -1) {
            constraints.remove(idx);
        }
    }

    /**
   * Raycast test
   * @deprecated Use .raycastAll, .raycastClosest or .raycastAny instead.
   */
    public void rayTest(Vec3 from, Vec3 to, Object result) {
        if (result instanceof RaycastResult) {
            // Do raycastClosest
            raycastClosest(from, to, /*added rayoptions constructor */new RayOptions(null , null , RayModes.ANY , null , true , -1,-1,true , null), (RaycastResult) result);
        } else {
            // Do raycastAll
            raycastAll(from, to, new RayOptions(null , null , RayModes.ANY , null , true , -1,-1,true , null), (RaycastCallback) result);
        }
    }
    
    /**
   * Ray cast against all bodies. The provided callback will be executed for each hit with a RaycastResult as single argument.
   * @return True if any body was hit.
   */
    public boolean raycastAll(Vec3 from, Vec3 to, RayOptions options, RaycastCallback callback) {
        options.setMode(RayModes.ALL); // Ray.ALL
        options.setFrom(from);
        options.setTo(to);
        options.setCallback(callback);
        return tmpRay.intersectWorld(this, options);
    }
    
    /**
   * Ray cast, and stop at the first result. Note that the order is random - but the method is fast.
   * @return True if any body was hit.
   */
    public boolean raycastAny(Vec3 from, Vec3 to, RayOptions options, RaycastResult result) {
        options.setMode(RayModes.ANY);
        options.setFrom(from);
        options.setTo(to);
        options.setResult(result);
        return tmpRay.intersectWorld(this, options);
    }

    /**
   * Ray cast, and return information of the closest hit.
   * @return True if any body was hit.
   */
    public boolean raycastClosest(Vec3 from, Vec3 to, RayOptions options, RaycastResult result) {
        //changed Ray.closest to RayModes.CLOSEST
        options.setMode(RayModes.CLOSEST);
        options.setFrom(from);
        options.setTo(to);
        options.setResult(result);
        return tmpRay.intersectWorld(this, options);
    }
    
    /**
   * Add a rigid body to the simulation.
   * @todo If the simulation has not yet started, why recrete and copy arrays for each body? Accumulate in dynamic arrays in this case.
   * @todo Adding an array of bodies should be possible. This would save some loops too
   */
    public void addBody(Body body) {
        if (!bodies.contains(body)) {
            body.index = (bodies.size());
            bodies.add(body);
            body.world = (this);
            body.initPosition.copy(body.position);
            body.initVelocity.copy(body.velocity);
            body.timeLastSleepy=time;
            
            if (body instanceof Body) {
                body.initAngularVelocity.copy(body.angularVelocity);
                body.initQuaternion.copy(body.quaternion);
            }
            
            collisionMatrix.setNumObjects(bodies.size());
            addBodyEvent.body = body;
            idToBodyMap.put(body.id, body);
            dispatchEvent(addBodyEvent);
        }
    }
    
    /**
   * Remove a rigid body from the simulation.
   */
    public void removeBody(Body body) {
        body.world = null;
        int n = bodies.size() - 1;
        List<Body> bodies = this.bodies;
        int idx = bodies.indexOf(body);
        
        if (idx != -1) {
            bodies.remove(idx);
            
            // Recompute index
            for (int i = 0; i < bodies.size(); i++) {
                bodies.get(i).index= i;
            }
            
            collisionMatrix.setNumObjects(n);
            removeBodyEvent.body = body;
            idToBodyMap.remove(body.id);
            dispatchEvent(removeBodyEvent);
        }
    }
    
    public Body getBodyById(int id) {
        return idToBodyMap.get(id);
    }

    /**
   * @todo Make a faster map
   */
    public Shape getShapeById(int id) {
        for (Body body : bodies) {
            for (Shape shape : body.shapes) {
                if (shape.id == id) {
                    return shape;
                }
            }
        }
        return null;
    }

    /**
   * Adds a contact material to the World
   */
    public void addContactMaterial(ContactMaterial cmat) {
        contactmaterials.add(cmat);
        //changed this
        contactMaterialTable.set(cmat.materials[0].id, cmat.materials[1].id, cmat);
    }

    /**
   * Removes a contact material from the World.
   */
    public void removeContactMaterial(ContactMaterial cmat) {
        int idx = contactmaterials.indexOf(cmat);
        if (idx != -1) {
            contactmaterials.remove(idx);
            //changed this
            contactMaterialTable.delete(cmat.materials[0].id, cmat.materials[1].id);
        }
    }

    /**
   * Step the simulation forward keeping track of last called time
   * to be able to step the world at a fixed rate, independently of framerate.
   *
   * @param dt The fixed time step size to use (default: 1 / 60).
   * @param maxSubSteps Maximum number of fixed steps to take per function call (default: 10).
   * @see https://gafferongames.com/post/fix_your_timestep/
   * @example
   *     // Run the simulation independently of framerate every 1 / 60 ms
   *     world.fixedStep()
   */
    public void fixedStep(double dt, int maxSubSteps) {
        double time = System.currentTimeMillis() / 1000.0; // seconds
        if (lastCallTime == 0) {
            this.step(dt, 0, maxSubSteps);
        } else {
            double timeSinceLastCalled = time - lastCallTime;
            this.step(dt, timeSinceLastCalled, maxSubSteps);
        }
        lastCallTime = time;
    }

    /**
   * Step the physics world forward in time.
   *
   * There are two modes. The simple mode is fixed timestepping without interpolation. In this case you only use the first argument. The second case uses interpolation. In that you also provide the time since the function was last used, as well as the maximum fixed timesteps to take.
   *
   * @param dt The fixed time step size to use.
   * @param timeSinceLastCalled The time elapsed since the function was last called.
   * @param maxSubSteps Maximum number of fixed steps to take per function call (default: 10).
   * @see https://web.archive.org/web/20180426154531/http://bulletphysics.org/mediawiki-1.5.8/index.php/Stepping_The_World#What_do_the_parameters_to_btDynamicsWorld::stepSimulation_mean.3F
   * @example
   *     // fixed timestepping without interpolation
   *     world.step(1 / 60)
   */
    public void step(double dt, double timeSinceLastCalled, int maxSubSteps) {
        if (timeSinceLastCalled == 0) {
            // Fixed, simple stepping
            internalStep(dt);

            // Increment time
            this.time += dt;
        } else {
            accumulator += timeSinceLastCalled;
            double t0 = System.currentTimeMillis();
            int substeps = 0;
            while (accumulator >= dt && substeps < maxSubSteps) {
                // Do fixed steps to catch up
                internalStep(dt);
                accumulator -= dt;
                substeps++;
                if (System.currentTimeMillis() - t0 > dt * 1000) {
                    // The framerate is not interactive anymore.
                    // We are below the target framerate.
                    // Better bail out.
                    break;
                }
            }

            // Remove the excess accumulator, since we may not
            // have had enough substeps available to catch up
            accumulator = accumulator % dt;

            double t = accumulator / dt;
            for (Body b : bodies) {
                b.previousPosition.lerp(b.position, t, b.interpolatedPosition);
                b.previousQuaternion.slerp(b.quaternion, t, b.interpolatedQuaternion);
                b.previousQuaternion.normalize();
            }
            this.time += timeSinceLastCalled;
        }
    }

    
    public void internalStep(double dt) {
        this.dt = dt;
        World world = this;
        World that = this;
        List<ContactEquation> contacts = this.contacts;
        List<Body> p1 = World_step_p1;
        List<Body> p2 = World_step_p2;
        int N = this.bodies.size();
        List<Body> bodies = this.bodies;
        Solver solver = this.solver;
        Vec3 gravity = this.gravity;
        boolean doProfiling = this.doProfiling;
        Profile profile = this.profile;
        int DYNAMIC = Body.DYNAMIC;
        double profilingStart = Double.NEGATIVE_INFINITY;
        List<Constraint> constraints = this.constraints;
        List<FrictionEquation> frictionEquationPool = World_step_frictionEquationPool;
        double gnorm = gravity.length();
        double gx = gravity.x;
        double gy = gravity.y;
        double gz = gravity.z;
        int i = 0;

        if (doProfiling) {
            profilingStart = System.nanoTime() / 1e6;
        }

        // Add gravity to all objects
        for (i = 0; i < N; i++) {
            Body bi = bodies.get(i);
            if (bi.type == DYNAMIC) {
                // Only for dynamic bodies
                Vec3 f = bi.force;
                double m = bi.mass;
                f.x += m * gx;
                f.y += m * gy;
                f.z += m * gz;
            }
        }

        // Update subsystems
        for (i = 0; i < this.subsystems.size(); i++) {
            this.subsystems.get(i).update();
        }

        // Collision detection
        if (doProfiling) {
            profilingStart = System.nanoTime() / 1e6;
        }
        p1.clear(); // Clean up pair arrays from the last step
        p2.clear();
        this.broadphase.collisionPairs(this, p1, p2);
        if (doProfiling) {
            profile.broadphase = System.nanoTime() / 1e6 - profilingStart;
        }

        // Remove constrained pairs with collideConnected == false
        int Nconstraints = constraints.size();
        for (i = 0; i < Nconstraints; i++) {
            Constraint c = constraints.get(i);
            if (!c.collideConnected) {
                for (int j = p1.size() - 1; j >= 0; j--) {
                    if ((c.bodyA == p1.get(j) && c.bodyB == p2.get(j)) || (c.bodyB == p1.get(j) && c.bodyA == p2.get(j))) {
                        p1.remove(j);
                        p2.remove(j);
                    }
                }
            }
        }

        this.collisionMatrixTick();

        // Generate contacts
        if (doProfiling) {
            profilingStart = System.nanoTime() / 1e6;
        }
        List<ContactEquation> oldcontacts = World_step_oldContacts;
        int NoldContacts = contacts.size();

        for (i = 0; i < NoldContacts; i++) {
            oldcontacts.add(contacts.get(i));
        }
        contacts.clear();

        // Transfer FrictionEquation from the current list to the pool for reuse
        int NoldFrictionEquations = this.frictionEquations.size();
        for (i = 0; i < NoldFrictionEquations; i++) {
            frictionEquationPool.add(this.frictionEquations.get(i));
        }
        this.frictionEquations.clear();

        this.narrowphase.getContacts(
            p1,
            p2,
            this,
            contacts,
            oldcontacts, // To be reused
            this.frictionEquations,
            frictionEquationPool
        );

        if (doProfiling) {
            profile.narrowphase = System.nanoTime() / 1e6 - profilingStart;
        }

        // Loop over all collisions
        if (doProfiling) {
            profilingStart = System.nanoTime() / 1e6;
        }

        // Add all friction equations
        for (i = 0; i < this.frictionEquations.size(); i++) {
            solver.addEquation(this.frictionEquations.get(i));
        }

        int ncontacts = contacts.size();
        for (int k = 0; k < ncontacts; k++) {
            // Current contact
            ContactEquation c = contacts.get(k);

            // Get current collision indices
            Body bi = c.bi;
            Body bj = c.bj;
            Shape si = c.si;
            Shape sj = c.sj;

            // Get collision properties
            ContactMaterial cm;
            if (bi.material != null && bj.material != null) {
                cm = this.getContactMaterial(bi.material, bj.material);
                if (cm == null) {
                    cm = this.defaultContactMaterial;
                }
            } else {
                cm = this.defaultContactMaterial;
            }

            double mu = cm.friction;

            // If friction or restitution were specified in the material, use them
            if (bi.material != null && bj.material != null) {
                if (bi.material.friction >= 0 && bj.material.friction >= 0) {
                    mu = bi.material.friction * bj.material.friction;
                }

                if (bi.material.restitution >= 0 && bj.material.restitution >= 0) {
                    c.restitution = bi.material.restitution * bj.material.restitution;
                }
            }

            solver.addEquation(c);

            if (bi.allowSleep && bi.type == Body.DYNAMIC && bi.sleepState == Body.SLEEPING && bj.sleepState == Body.AWAKE && bj.type != Body.STATIC) {
                double speedSquaredB = bj.velocity.lengthSquared() + bj.angularVelocity.lengthSquared();
                double speedLimitSquaredB = bj.sleepSpeedLimit * bj.sleepSpeedLimit;
                if (speedSquaredB >= speedLimitSquaredB * 2) {
                    bi.wakeUpAfterNarrowphase = true;
                }
            }

            if (bj.allowSleep && bj.type == Body.DYNAMIC && bj.sleepState == Body.SLEEPING && bi.sleepState == Body.AWAKE && bi.type != Body.STATIC) {
                double speedSquaredA = bi.velocity.lengthSquared() + bi.angularVelocity.lengthSquared();
                double speedLimitSquaredA = bi.sleepSpeedLimit * bi.sleepSpeedLimit;
                if (speedSquaredA >= speedLimitSquaredA * 2) {
                    bj.wakeUpAfterNarrowphase = true;
                }
            }

            // Now we know that i and j are in contact. Set the collision matrix state
            this.collisionMatrix.set(bi, bj, true);

            if (!this.collisionMatrixPrevious.get(bi, bj)) {
                // First contact!
                // We reuse the collideEvent object, otherwise, we will end up creating new objects for each new contact, even if there's no event listener attached.
                World_step_collideEvent.body = bj;
                World_step_collideEvent.contact = c;
                bi.dispatchEvent(World_step_collideEvent);

                World_step_collideEvent.body = bi;
                bj.dispatchEvent(World_step_collideEvent);
            }

            this.bodyOverlapKeeper.set(bi.id, bj.id);
            this.shapeOverlapKeeper.set(si.id, sj.id);
        }

        this.emitContactEvents();

        if (doProfiling) {
            profile.makeContactConstraints = System.nanoTime() / 1e6 - profilingStart;
            profilingStart = System.nanoTime() / 1e6;
        }

        // Wake up bodies
        for (i = 0; i < N; i++) {
            Body bi = bodies.get(i);
            if (bi.wakeUpAfterNarrowphase) {
                bi.wakeUp();
                bi.wakeUpAfterNarrowphase = false;
            }
        }

        // Add user-added constraints
        Nconstraints = constraints.size();
        for (i = 0; i < Nconstraints; i++) {
            Constraint c = constraints.get(i);
            c.update();
            List<Equation> equations = c.equations;
            for (int j = 0, Neq = equations.size(); j < Neq; j++) {
                Equation eq = equations.get(j);
                solver.addEquation(eq);
            }
        }

        // Solve the constrained system
        solver.solve(dt, this);

        if (doProfiling) {
            profile.solve = System.nanoTime() / 1e6 - profilingStart;
        }

        // Remove all contacts from the solver
        solver.removeAllEquations();

        // Apply damping
        for (i = 0; i < N; i++) {
            Body bi = bodies.get(i);
            if ((bi.type & Body.DYNAMIC) != 0) {
                double ld = Math.pow(1.0 - bi.linearDamping, dt);
                Vec3 v = bi.velocity;
                v.scale(ld, v);
                Vec3 av = bi.angularVelocity;
                if (av != null) {
                    double ad = Math.pow(1.0 - bi.angularDamping, dt);
                    av.scale(ad, av);
                }
            }
        }

        this.dispatchEvent(World_step_preStepEvent);

        // Leap frog
        if (doProfiling) {
            profilingStart = System.nanoTime() / 1e6;
        }
        int stepnumber = this.stepnumber;
        boolean quatNormalize = stepnumber % (this.quatNormalizeSkip + 1) == 0;
        boolean quatNormalizeFast = this.quatNormalizeFast;

        for (i = 0; i < N; i++) {
            bodies.get(i).integrate(dt, quatNormalize, quatNormalizeFast);
        }
        this.clearForces();

        this.broadphase.dirty = true;

        if (doProfiling) {
            profile.integrate = System.nanoTime() / 1e6 - profilingStart;
        }

        // Update step number
        this.stepnumber += 1;

        this.dispatchEvent(World_step_postStepEvent);

        // Sleeping update
        boolean hasActiveBodies = true;
        if (this.allowSleep) {
            hasActiveBodies = false;
            for (i = 0; i < N; i++) {
                Body bi = bodies.get(i);
                bi.sleepTick(this.time);

                if (bi.sleepState != Body.SLEEPING) {
                    hasActiveBodies = true;
                }
            }
        }
        this.hasActiveBodies = hasActiveBodies;
    }
    
    public void emitContactEvents() {
        boolean hasBeginContact = this.hasAnyEventListener("beginContact");
        boolean hasEndContact = this.hasAnyEventListener("endContact");

        List<Integer> additions = new ArrayList<>();
        List<Integer> removals = new ArrayList<>();

        if (hasBeginContact || hasEndContact) {
            this.bodyOverlapKeeper.getDiff(additions, removals);
        }

        if (hasBeginContact) {
            for (int i = 0; i < additions.size(); i += 2) {
                int bodyIdA = additions.get(i);
                int bodyIdB = additions.get(i + 1);
                Body bodyA = this.getBodyById(bodyIdA);
                Body bodyB = this.getBodyById(bodyIdB);
                beginContactEvent.bodyA = bodyA;
                beginContactEvent.bodyB = bodyB;
                this.dispatchEvent(beginContactEvent);
            }
            beginContactEvent.bodyA = beginContactEvent.bodyB = null;
        }

        if (hasEndContact) {
            for (int i = 0; i < removals.size(); i += 2) {
                int bodyIdA = removals.get(i);
                int bodyIdB = removals.get(i + 1);
                Body bodyA = this.getBodyById(bodyIdA);
                Body bodyB = this.getBodyById(bodyIdB);
                endContactEvent.bodyA = bodyA;
                endContactEvent.bodyB = bodyB;
                this.dispatchEvent(endContactEvent);
            }
            endContactEvent.bodyA = endContactEvent.bodyB = null;
        }

        additions.clear();
        removals.clear();

        boolean hasBeginShapeContact = this.hasAnyEventListener("beginShapeContact");
        boolean hasEndShapeContact = this.hasAnyEventListener("endShapeContact");

        if (hasBeginShapeContact || hasEndShapeContact) {
            this.shapeOverlapKeeper.getDiff(additions, removals);
        }

        if (hasBeginShapeContact) {
            for (int i = 0; i < additions.size(); i += 2) {
                int shapeIdA = additions.get(i);
                int shapeIdB = additions.get(i + 1);
                Shape shapeA = this.getShapeById(shapeIdA);
                Shape shapeB = this.getShapeById(shapeIdB);
                beginShapeContactEvent.shapeA = shapeA;
                beginShapeContactEvent.shapeB = shapeB;
                if (shapeA != null) beginShapeContactEvent.bodyA = shapeA.body;
                if (shapeB != null) beginShapeContactEvent.bodyB = shapeB.body;
                this.dispatchEvent(beginShapeContactEvent);
            }
            beginShapeContactEvent.bodyA = beginShapeContactEvent.bodyB = beginShapeContactEvent.shapeA = beginShapeContactEvent.shapeB = null;
        }

        if (hasEndShapeContact) {
            for (int i = 0; i < removals.size(); i += 2) {
                int shapeIdA = removals.get(i);
                int shapeIdB = removals.get(i + 1);
                Shape shapeA = this.getShapeById(shapeIdA);
                Shape shapeB = this.getShapeById(shapeIdB);
                endShapeContactEvent.shapeA = shapeA;
                endShapeContactEvent.shapeB = shapeB;
                if (shapeA != null) endShapeContactEvent.bodyA = shapeA.body;
                if (shapeB != null) endShapeContactEvent.bodyB = shapeB.body;
                this.dispatchEvent(endShapeContactEvent);
            }
            endShapeContactEvent.bodyA = endShapeContactEvent.bodyB = endShapeContactEvent.shapeA = endShapeContactEvent.shapeB = null;
        }
    }

    /**
   * Sets all body forces in the world to zero.
   */
    public void clearForces() {
        List<Body> bodies = this.bodies;
        int N = bodies.size();
        for (int i = 0; i < N; i++) {
            Body b = bodies.get(i);
            Vec3 force = b.force;
            Vec3 torque = b.torque;

            force.set(0, 0, 0);
            torque.set(0, 0, 0);
        }
    }
    
    
 // Temp stuff
    private AABB tmpAABB1 = new AABB();
    private List<Object> tmpArray1 = new ArrayList<>();
    private Ray tmpRay = new Ray();

    // performance.now() fallback on System.currentTimeMillis()
    //private Performance performance = new Performance();

    public World() {
        long nowOffset = System.currentTimeMillis();
        this.performance.now = () -> System.currentTimeMillis() - nowOffset;
    }

    private Vec3 step_tmp1 = new Vec3();

    // Dispatched after the world has stepped forward in time.
    // Reusable event objects to save memory.
    private Event World_step_postStepEvent = new Event("postStep");

    // Dispatched before the world steps forward in time.
    private Event World_step_preStepEvent = new Event("preStep");

    private Event World_step_collideEvent = new Event(Body.COLLIDE_EVENT_NAME);

    // Pools for unused objects
    private List<ContactEquation> World_step_oldContacts = new ArrayList<>();
    private List<FrictionEquation> World_step_frictionEquationPool = new ArrayList<>();

    // Reusable arrays for collision pairs
    private List<Body> World_step_p1 = new ArrayList<>();
    private List<Body> World_step_p2 = new ArrayList<>();

    // Stuff for emitContactEvents
    private List<Integer> additions = new ArrayList<>();
    private List<Integer> removals = new ArrayList<>();

    private class ContactEvent {
        private String type;
        private Body bodyA;
        private Body bodyB;

        public ContactEvent(String type) {
            this.type = type;
            this.bodyA = null;
            this.bodyB = null;
        }
    }

    private ContactEvent beginContactEvent = new ContactEvent("beginContact");
    private ContactEvent endContactEvent = new ContactEvent("endContact");

    private class ShapeContactEvent {
        private String type;
        private Body bodyA;
        private Body bodyB;
        private Shape shapeA;
        private Shape shapeB;

        public ShapeContactEvent(String type) {
            this.type = type;
            this.bodyA = null;
            this.bodyB = null;
            this.shapeA = null;
            this.shapeB = null;
        }
    }

    private ShapeContactEvent beginShapeContactEvent = new ShapeContactEvent("beginShapeContact");
    private ShapeContactEvent endShapeContactEvent = new ShapeContactEvent("endShapeContact");
}


