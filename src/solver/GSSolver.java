package solver;

import java.util.List;

import math.Vec3; 
import objects.Body;

import world.World;
import equations.Equation;

/**
 * Constraint equation Gauss-Seidel solver.
 * @todo The spook parameters should be specified for each constraint, not globally.
 * @see https://www8.cs.umu.se/kurser/5DV058/VT09/lectures/spooknotes.pdf
 */
public class GSSolver extends Solver {
    // The number of solver iterations determines the quality of constraints in the world.
    // More iterations yield a more correct simulation but require more computations.
    // Adjust this value based on your world's characteristics.
    int iterations;

    /**
   * When tolerance is reached, the system is assumed to be converged.
   */
    double tolerance;

    // Constructor
    public GSSolver() {
        super();

        // Initialize solver parameters
        this.iterations = 10;
        this.tolerance = 1e-7;
    }

    // Solve method
    @Override
    public int solve(double dt, World world) {
        int iter = 0;
        final int maxIter = this.iterations;
        final double tolSquared = this.tolerance * this.tolerance;
        final List<Equation> equations = this.equations;
        final int Neq = equations.size();
        final List<Body> bodies = world.bodies;
        final int Nbodies = bodies.size();
        final double h = dt;
        double q;
        double B;
        double invC;
        double deltalambda;
        double deltalambdaTot;
        double GWlambda;
        double lambdaj;

        // Update solve mass
        if (Neq != 0) {
            for (int i = 0; i < Nbodies; i++) {
                bodies.get(i).updateSolveMassProperties();
            }
        }

        // Initialize temporary arrays for values that don't change during iteration
        double[] invCs = new double[Neq];
        double[] Bs = new double[Neq];
        double[] lambda = new double[Neq];

        for (int i = 0; i < Neq; i++) {
            Equation c = equations.get(i);
            lambda[i] = 0.0;
            Bs[i] = c.computeB(h);
            invCs[i] = 1.0 / c.computeC();
        }

        if (Neq != 0) {
            // Reset vlambda
            for (int i = 0; i < Nbodies; i++) {
                Body b = bodies.get(i);
                Vec3 vlambda = b.vlambda;
                Vec3 wlambda = b.wlambda;
                vlambda.set(0, 0, 0);
                wlambda.set(0, 0, 0);
            }

            // Iterate over equations
            for (iter = 0; iter < maxIter; iter++) {
                // Accumulate the total error for each iteration.
                deltalambdaTot = 0.0;

                for (int j = 0; j < Neq; j++) {
                    Equation c = equations.get(j);

                    // Compute iteration
                    B = Bs[j];
                    invC = invCs[j];
                    lambdaj = lambda[j];
                    GWlambda = c.computeGWlambda();
                    deltalambda = invC * (B - GWlambda - c.eps * lambdaj);

                    // Clamp if we are not within the min/max interval
                    if (lambdaj + deltalambda < c.minForce) {
                        deltalambda = c.minForce - lambdaj;
                    } else if (lambdaj + deltalambda > c.maxForce) {
                        deltalambda = c.maxForce - lambdaj;
                    }
                    lambda[j] += deltalambda;

                    deltalambdaTot += Math.abs(deltalambda);

                    c.addToWlambda(deltalambda);
                }

                // If the total error is small enough, stop iterating
                if (deltalambdaTot * deltalambdaTot < tolSquared) {
                    break;
                }
            }

            // Add the result to velocity
            for (int i = 0; i < Nbodies; i++) {
                Body b = bodies.get(i);
                Vec3 v = b.velocity;
                Vec3 w = b.angularVelocity;

                b.vlambda.vmul(b.linearFactor, b.vlambda);
                v.vadd(b.vlambda, v);

                b.wlambda.vmul(b.angularFactor, b.wlambda);
                w.vadd(b.wlambda, w);
            }

            // Set the `.multiplier` property of each equation
            int l = equations.size();
            double invDt = 1.0 / h;
            while (l-- > 0) {
                equations.get(l).multiplier = lambda[l] * invDt;
            }
        }

        return iter;
    }
}

