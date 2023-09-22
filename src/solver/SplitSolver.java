package solver;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import java.util.Queue;

import equations.Equation;
import world.World;
import objects.Body;

/**
 * Splits the equations into islands and solves them independently. Can improve
 * performance.
 */
public class SplitSolver extends Solver {
    /**
     * The number of solver iterations determines quality of the constraints in the
     * world. The more iterations, the more correct simulation. More iterations need
     * more computations though. If you have a large gravity force in your world,
     * you will need more iterations.
     */
    private int iterations;
    /**
     * When tolerance is reached, the system is assumed to be converged.
     */
    private double tolerance;
    private GSSolver subsolver;
    private List<SplitSolverNode> nodes;
    private List<SplitSolverNode> nodePool;

    // Constructor
    public SplitSolver(GSSolver subsolver) {
        super();
        this.iterations = 10;
        this.tolerance = 1e-7;
        this.subsolver = subsolver;
        this.nodes = new ArrayList<>();
        this.nodePool = new ArrayList<>();

        // Create needed nodes, reuse if possible
        while (this.nodePool.size() < 128) {
            this.nodePool.add(createNode());
        }
    }

    // Creates a new SplitSolverNode
    public SplitSolverNode createNode() {
        return new SplitSolverNode(null, new ArrayList<>(), new ArrayList<>(), false);
    }

    // Override the solve method
    @Override
    public int solve(double dt, World world) {
        List<SplitSolverNode> nodes = new ArrayList<>();
        List<SplitSolverNode> nodePool = this.nodePool;
        List<Body> bodies = world.bodies;
        List<Equation> equations = this.equations;
        int Neq = equations.size();
        int Nbodies = bodies.size();
        GSSolver subsolver = this.subsolver;

        // Create needed nodes, reuse if possible
        while (nodePool.size() < Nbodies) {
            nodePool.add(createNode());
        }
        nodes.addAll(nodePool.subList(0, Nbodies));

        // Reset node values
        for (int i = 0; i < Nbodies; i++) {
            SplitSolverNode node = nodes.get(i);
            node.setBody(bodies.get(i));
            node.getChildren().clear();
            node.getEquations().clear();
            node.setVisited(false);
        }
        for (int k = 0; k < Neq; k++) {
            Equation eq = equations.get(k);
            int i = bodies.indexOf(eq.bi);
            int j = bodies.indexOf(eq.bj);
            SplitSolverNode ni = nodes.get(i);
            SplitSolverNode nj = nodes.get(j);
            ni.getChildren().add(nj);
            ni.getEquations().add(eq);
            nj.getChildren().add(ni);
            nj.getEquations().add(eq);
        }

        int n = 0;
        List<Equation> eqs = new ArrayList<>();

        subsolver.tolerance = (this.tolerance);
        subsolver.iterations = (this.iterations);

        World dummyWorld = new World();
        while (true) {
            SplitSolverNode child = getUnvisitedNode(nodes);
            if (child == null) {
                break;
            }

            eqs.clear();
            dummyWorld.bodies.clear();
            bfs(child, /*visitFunc,*/ dummyWorld.bodies, eqs);

            int Neqs = eqs.size();

            eqs.sort((a, b) -> b.id - a.id);

            for (int i = 0; i < Neqs; i++) {
                subsolver.addEquation(eqs.get(i));
            }

            int iter = subsolver.solve(dt, dummyWorld);
            subsolver.removeAllEquations();
            n++;
        }

        return n;
    }

    // Returns an unvisited node from the list of nodes
    private SplitSolverNode getUnvisitedNode(List<SplitSolverNode> nodes) {
        int Nnodes = nodes.size();
        for (int i = 0; i < Nnodes; i++) {
            SplitSolverNode node = nodes.get(i);
            if (!node.isVisited() && (node.getBody() != null) && (node.getBody().type != Body.STATIC)) {
                return node;
            }
        }
        return null;
    }

    // Breadth-first search to traverse nodes and collect equations
    private void bfs(SplitSolverNode root, List<Body> bds, List<Equation> eqs) {
        Queue<SplitSolverNode> queue = new LinkedList<>();
        queue.add(root);
        root.setVisited(true);
       visitFunc(root, bds, eqs);

        while (!queue.isEmpty()) {
            SplitSolverNode node = queue.poll();
            for (SplitSolverNode child : getUnvisitedChildren(node)) {
                child.setVisited(true);
                visitFunc(child, bds, eqs);
                queue.add(child);
            }
        }
    }

    // Returns unvisited children of a node
    private List<SplitSolverNode> getUnvisitedChildren(SplitSolverNode node) {
        List<SplitSolverNode> unvisitedChildren = new ArrayList<>();
        for (SplitSolverNode child : node.getChildren()) {
            if (!child.isVisited()) {
                unvisitedChildren.add(child);
            }
        }
        return unvisitedChildren;
    }

    public void visitFunc(SplitSolverNode node, List<Body> bds, List<Equation> eqs) {
        bds.add(node.getBody());
        int Neqs = node.getEquations().size();
        for (int i = 0; i < Neqs; i++) {
            Equation eq = node.getEquations().get(i);
            if (!eqs.contains(eq)) {
                eqs.add(eq);
            }
        }
    }

    // Functional interface for visiting nodes and collecting equations
    private interface VisitFunc {
        void visit(SplitSolverNode node, List<Body> bds, List<Equation> eqs);
    }

    // Inner class representing a node in the SplitSolver tree

}
