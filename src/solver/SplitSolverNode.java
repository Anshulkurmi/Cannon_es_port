package solver;

import java.util.List;

import equations.Equation;
import objects.Body;

public class SplitSolverNode {
        private Body body;
        private List<SplitSolverNode> children;
        private List<Equation> eqs;
        private boolean visited;

        public SplitSolverNode(Body body, List<SplitSolverNode> children, List<Equation> eqs, boolean visited) {
            this.body = body;
            this.children = children;
            this.eqs = eqs;
            this.visited = visited;
        }

        public Body getBody() {
            return body;
        }

        public void setBody(Body body) {
            this.body = body;
        }

        public List<SplitSolverNode> getChildren() {
            return children;
        }

        public List<Equation> getEquations() {
            return eqs;
        }

        public boolean isVisited() {
            return visited;
        }

        public void setVisited(boolean visited) {
            this.visited = visited;
        }
    }
