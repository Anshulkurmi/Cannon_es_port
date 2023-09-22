package equations;

import math.Vec3;
import objects.Body;
import math.JacobianElement;

public class FrictionEquation extends Equation {
    public Vec3 ri;
    public Vec3 rj;
    public Vec3 t; // Tangent

    /**
     * @param slipForce should be +-F_friction = +-mu * F_normal = +-mu * m * g
     */
    public FrictionEquation(Body bodyA, Body bodyB, double slipForce) {
        super(bodyA, bodyB, -slipForce, slipForce);
        this.ri = new Vec3();
        this.rj = new Vec3();
        this.t = new Vec3();
    }

    @Override
    public double computeB(double h) {
        double a = this.a;
        double b = this.b;
        Body bi = this.bi;
        Body bj = this.bj;
        Vec3 ri = this.ri;
        Vec3 rj = this.rj;
        Vec3 rixt = FrictionEquation_computeB_temp1;
        Vec3 rjxt = FrictionEquation_computeB_temp2;
        Vec3 t = this.t;

        // Calculate cross products
        ri.cross(t, rixt);
        rj.cross(t, rjxt);

        // G = [-t -rixt t rjxt]
        // And remember, this is a pure velocity constraint, g is always zero!
        JacobianElement GA = this.jacobianElementA;
        JacobianElement GB = this.jacobianElementB;

        t.negate(GA.spatial);
        rixt.negate(GA.rotational);
        GB.spatial.copy(t);
        GB.rotational.copy(rjxt);

        double GW = this.computeGW();
        double GiMf = this.computeGiMf();

        double B = -GW * b - h * GiMf;

        return B;
    }
    //added
    static final Vec3 FrictionEquation_computeB_temp1 = new Vec3();
    static final Vec3 FrictionEquation_computeB_temp2 = new Vec3();

}
