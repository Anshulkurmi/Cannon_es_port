package collision;

import java.util.HashMap;
import java.util.Map;

import objects.Body;

/**
 * Records what objects are colliding with each other
 */
public class ObjectCollisionMatrix {
    private Map<String, Boolean> matrix; // The matrix storage.

    public ObjectCollisionMatrix() {
        this.matrix = new HashMap<>();
    }

    /**
     * Check if two bodies are colliding.
     */
    public boolean get(Body bi, Body bj) {
        int i = bi.id;
        int j = bj.id;
        if (j > i) {
            int temp = j;
            j = i;
            i = temp;
        }
        return this.matrix.containsKey(i + "-" + j);
    }

    /**
     * Set collision status between two bodies.
     */
    public void set(Body bi, Body bj, boolean value) {
        int i = bi.id;
        int j = bj.id;
        if (j > i) {
            int temp = j;
            j = i;
            i = temp;
        }
        if (value) {
            this.matrix.put(i + "-" + j, true);
        } else {
            this.matrix.remove(i + "-" + j);
        }
    }

    /**
     * Empty the matrix.
     */
    public void reset() {
        this.matrix.clear();
    }

    /**
     * Set the maximum number of objects (no-op in this implementation).
     */
    public void setNumObjects(int n) {
        // This method is a no-op in this implementation
    }
}
