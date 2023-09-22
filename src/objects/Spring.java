package objects;

import math.Vec3;

/**
 * A spring, connecting two bodies.
 * 
 * @example
 *          const spring = new Spring(boxBody, sphereBody, {
 *          restLength: 0,
 *          stiffness: 50,
 *          damping: 1,
 *          })
 *
 *          // Compute the force after each step
 *          world.addEventListener('postStep', (event) => {
 *          spring.applyForce()
 *          })
 */
public class Spring {
    // Properties
    /**
     * Rest length of the spring. A number > 0.
     * 
     * @default 1
     */
    private double restLength;

    /**
     * Stiffness of the spring. A number >= 0.
     * 
     * @default 100
     */
    private double stiffness;
    /**
     * Damping of the spring. A number >= 0.
     * 
     * @default 1
     */
    private double damping;
    /**
     * First connected body.
     */
    private Body bodyA;
    private Body bodyB;
    /**
     * Anchor for bodyA in local bodyA coordinates.
     * Where to hook the spring to body A, in local body coordinates.
     * 
     * @default new Vec3()
     */
    private Vec3 localAnchorA;
    private Vec3 localAnchorB;

    // Constructor
    public Spring(Body bodyA, Body bodyB, SpringOptions options) {
        // Initialize properties with default values if not provided
        // changed
        this.restLength = options.restLength;// (options.restLength != null) ? options.restLength : 1.0;
        this.stiffness = options.stiffness;// (options.stiffness != null) ? options.stiffness : 100.0;
        this.damping = options.damping;// (options.damping != null) ? options.damping : 1.0;
        this.bodyA = bodyA;
        this.bodyB = bodyB;
        this.localAnchorA = (options.localAnchorA != null) ? options.localAnchorA.clone() : new Vec3();
        this.localAnchorB = (options.localAnchorB != null) ? options.localAnchorB.clone() : new Vec3();

        // Set world anchors if provided
        if (options.worldAnchorA != null) {
            this.setWorldAnchorA(options.worldAnchorA);
        }
        if (options.worldAnchorB != null) {
            this.setWorldAnchorB(options.worldAnchorB);
        }
    }

    // Set the anchor point on body A using world coordinates
    public void setWorldAnchorA(Vec3 worldAnchorA) {
        this.bodyA.pointToLocalFrame(worldAnchorA, this.localAnchorA);
    }

    // Set the anchor point on body B using world coordinates
    public void setWorldAnchorB(Vec3 worldAnchorB) {
        this.bodyB.pointToLocalFrame(worldAnchorB, this.localAnchorB);
    }

    /**
     * Get the anchor point on body A, in world coordinates.
     * 
     * @param result The vector to store the result in.
     */
    public void getWorldAnchorA(Vec3 result) {
        this.bodyA.pointToWorldFrame(this.localAnchorA, result);
    }

    /**
     * Get the anchor point on body B, in world coordinates.
     * 
     * @param result The vector to store the result in.
     */
    public void getWorldAnchorB(Vec3 result) {
        this.bodyB.pointToWorldFrame(this.localAnchorB, result);
    }

    // Apply the spring force to connected bodies
    public void applyForce() {
        double k = this.stiffness;
        double d = this.damping;
        double l = this.restLength;
        Body bodyA = this.bodyA;
        Body bodyB = this.bodyB;
        Vec3 r = applyForce_r;
        Vec3 r_unit = applyForce_r_unit;
        Vec3 u = applyForce_u;
        Vec3 f = applyForce_f;
        Vec3 tmp = applyForce_tmp;
        Vec3 worldAnchorA = applyForce_worldAnchorA;
        Vec3 worldAnchorB = applyForce_worldAnchorB;
        Vec3 ri = applyForce_ri;
        Vec3 rj = applyForce_rj;
        Vec3 ri_x_f = applyForce_ri_x_f;
        Vec3 rj_x_f = applyForce_rj_x_f;

        // Get world anchor points
        this.getWorldAnchorA(worldAnchorA);
        this.getWorldAnchorB(worldAnchorB);

        // Compute offset points
        worldAnchorA.vsub(bodyA.position, ri);
        worldAnchorB.vsub(bodyB.position, rj);

        // Compute distance vector between world anchor points
        worldAnchorB.vsub(worldAnchorA, r);
        double rlen = r.length();
        r_unit.copy(r);
        r_unit.normalize();

        // Compute relative velocity of anchor points
        bodyB.velocity.vsub(bodyA.velocity, u);
        // Add rotational velocity
        bodyB.angularVelocity.cross(rj, tmp);
        u.vadd(tmp, u);
        bodyA.angularVelocity.cross(ri, tmp);
        u.vsub(tmp, u);

        // Calculate spring force
        r_unit.scale(-k * (rlen - l) - d * u.dot(r_unit), f);

        // Apply forces to bodies
        bodyA.force.vsub(f, bodyA.force);
        bodyB.force.vadd(f, bodyB.force);

        // Angular force
        ri.cross(f, ri_x_f);
        rj.cross(f, rj_x_f);
        bodyA.torque.vsub(ri_x_f, bodyA.torque);
        bodyB.torque.vadd(rj_x_f, bodyB.torque);
    }

    static Vec3 applyForce_r = new Vec3();
    static Vec3 applyForce_r_unit = new Vec3();
    static Vec3 applyForce_u = new Vec3();
    static Vec3 applyForce_f = new Vec3();
    static Vec3 applyForce_worldAnchorA = new Vec3();
    static Vec3 applyForce_worldAnchorB = new Vec3();
    static Vec3 applyForce_ri = new Vec3();
    static Vec3 applyForce_rj = new Vec3();
    static Vec3 applyForce_ri_x_f = new Vec3();
    static Vec3 applyForce_rj_x_f = new Vec3();
    static Vec3 applyForce_tmp = new Vec3();
}
