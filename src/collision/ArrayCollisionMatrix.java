package collision;

import objects.Body;

/**
 * Collision "matrix".
 * It's actually a triangular-shaped array of whether two bodies are touching this step, for reference next step
 */
public class ArrayCollisionMatrix {
    private int[] matrix; // The matrix storage.

    public ArrayCollisionMatrix() {
        matrix = new int[0]; // Initialize an empty matrix.
    }

    // Get an element
    public int get(Body bi, Body bj) {
        int i = bi.index;
        int j = bj.index;
        if (j > i) {
            int temp = j;
            j = i;
            i = temp;
        }
        return matrix[((i * (i + 1)) >> 1) + j - 1]; // Retrieve the value from the matrix.
    }

    // Set an element
    public void set(Body bi, Body bj, boolean value) {
        int i = bi.index;
        int j = bj.index;
        if (j > i) {
            int temp = j;
            j = i;
            i = temp;
        }
        matrix[((i * (i + 1)) >> 1) + j - 1] = value ? 1 : 0; // Set the value in the matrix.
    }

    // Sets all elements to zero
    public void reset() {
        for (int i = 0; i < matrix.length; i++) {
            matrix[i] = 0; // Reset all elements in the matrix to zero.
        }
    }

    // Sets the max number of objects
    public void setNumObjects(int n) {
        matrix = new int[(n * (n - 1)) >> 1]; // Resize the matrix based on the number of objects.
    }
}
