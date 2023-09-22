package solver;

import java.util.ArrayList;
import java.util.List;

import equations.Equation;
import world.World;

/**
 * Constraint equation solver base class.
 */
public class Solver {
    // List to store equations
    protected List<Equation> equations;

    // Constructor
    public Solver() {
        // Initialize the equations list
        this.equations = new ArrayList<>();
    }

    // Method to solve equations (to be implemented by subclasses)
    /**
   * Should be implemented in subclasses!
   * @todo use abstract
   * @return number of iterations performed
   */
    public int solve(double dt, World world) {
        return 0; // Should return the number of iterations done!
    }

    // Method to add an equation to the list
    public void addEquation(Equation eq) {
        // Check if the equation is enabled and neither body is a trigger
        if (eq.enabled && !eq.bi.isTrigger && !eq.bj.isTrigger) {
            equations.add(eq);
        }
    }

    // Method to remove an equation from the list
    public void removeEquation(Equation eq) {
        equations.remove(eq);
    }

    // Method to remove all equations from the list
    public void removeAllEquations() {
        equations.clear();
    }
}

