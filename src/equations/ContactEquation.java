package equations;

import objects.Body;

import math.JacobianElement;
import math.Vec3;

/**
 * Contact/non-penetration constraint equation
 */
public class ContactEquation extends Equation {
    /**
     * "bounciness": u1 = -e*u0
     */
    public double restitution;
    /**
     * World-oriented vector that goes from the center of bi to the contact point.
     */
    public Vec3 ri;
    /**
     * World-oriented vector that starts in body j position and goes to the contact
     * point.
     */
    public Vec3 rj;
    /**
     * Contact normal, pointing out of body i.
     */
    public Vec3 ni;

    public ContactEquation(Body bodyA, Body bodyB) {
        this(bodyA, bodyB, 1e6);
    }

    public ContactEquation(Body bodyA, Body bodyB, double maxForce) {
        super(bodyA, bodyB, 0, maxForce);

        this.restitution = 0.0;
        this.ri = new Vec3();
        this.rj = new Vec3();
        this.ni = new Vec3();
    }

    public double computeB(double h) {
        double a = this.a;
        double b = this.b;
        Body bi = this.bi;
        Body bj = this.bj;
        Vec3 ri = this.ri;
        Vec3 rj = this.rj;
        Vec3 rixn = ContactEquation_computeB_temp1;
        Vec3 rjxn = ContactEquation_computeB_temp2;
        Vec3 vi = bi.velocity;
        Vec3 wi = bi.angularVelocity;
        Vec3 fi = bi.force;
        Vec3 taui = bi.torque;
        Vec3 vj = bj.velocity;
        Vec3 wj = bj.angularVelocity;
        Vec3 fj = bj.force;
        Vec3 tauj = bj.torque;
        Vec3 penetrationVec = ContactEquation_computeB_temp3;
        JacobianElement GA = this.jacobianElementA ;
        JacobianElement GB = this.jacobianElementB ;

        Vec3 n = this.ni;

        // Calculate cross products
        ri.cross(n, rixn);
        rj.cross(n, rjxn);

        // g = xj + rj - (xi + ri)
        // G = [ -ni -rixn ni rjxn ]
        n.negate(GA.spatial);
        rixn.negate(GA.rotational);
        GB.spatial.copy(n);
        GB.rotational.copy(rjxn);

        // Calculate the penetration vector
        penetrationVec.copy(bj.position);
        penetrationVec.vadd(rj, penetrationVec);
        penetrationVec.vsub(bi.position, penetrationVec);
        penetrationVec.vsub(ri, penetrationVec);

        double g = n.dot(penetrationVec);

        // Compute iteration
        double ePlusOne = this.restitution + 1;
        double GW = ePlusOne * vj.dot(n) - ePlusOne * vi.dot(n) + wj.dot(rjxn) - wi.dot(rixn);
        double GiMf = this.computeGiMf();

        double B = -g * a - GW * b - h * GiMf;

        return B;
    }

    /**
   * Get the current relative velocity in the contact point.
   */
    public double getImpactVelocityAlongNormal() {
        Vec3 vi = ContactEquation_getImpactVelocityAlongNormal_vi;
        Vec3 vj = ContactEquation_getImpactVelocityAlongNormal_vj;
        Vec3 xi = ContactEquation_getImpactVelocityAlongNormal_xi;
        Vec3 xj = ContactEquation_getImpactVelocityAlongNormal_xj;
        Vec3 relVel = ContactEquation_getImpactVelocityAlongNormal_relVel;

        this.bi.position.vadd(this.ri, xi);
        this.bj.position.vadd(this.rj, xj);

        this.bi.getVelocityAtWorldPoint(xi, vi);
        this.bj.getVelocityAtWorldPoint(xj, vj);

        //relVel.copy(vi);
        vi.vsub(vj, relVel);

        return this.ni.dot(relVel);
    }

    static Vec3 ContactEquation_computeB_temp1 = new Vec3();// Temp vectors
    static Vec3 ContactEquation_computeB_temp2 = new Vec3();
    static Vec3 ContactEquation_computeB_temp3 = new Vec3();

    static Vec3 ContactEquation_getImpactVelocityAlongNormal_vi = new Vec3();
    static Vec3 ContactEquation_getImpactVelocityAlongNormal_vj = new Vec3();
    static Vec3 ContactEquation_getImpactVelocityAlongNormal_xi = new Vec3();
    static Vec3 ContactEquation_getImpactVelocityAlongNormal_xj = new Vec3();
    static Vec3 ContactEquation_getImpactVelocityAlongNormal_relVel = new Vec3();

}
