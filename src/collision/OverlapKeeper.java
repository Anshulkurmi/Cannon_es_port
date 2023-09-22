package collision;

import java.util.ArrayList;
import java.util.List;

public class OverlapKeeper {
    private List<Integer> current;  // List to store current overlap keys
    private List<Integer> previous; // List to store previous overlap keys

    public OverlapKeeper() {
        this.current = new ArrayList<>();
        this.previous = new ArrayList<>();
    }

    /**
     * Calculate a unique key for a pair of integers (i, j).
     * Ensures the key is consistent regardless of the order of i and j.
     */
    public int getKey(int i, int j) {
        if (j < i) {
            int temp = j;
            j = i;
            i = temp;
        }
        return (i << 16) | j;
    }

    /**
     * Add an overlap key for a pair of integers (i, j).
     * The keys are inserted in sorted order for efficient diff calculation.
     */
    public void set(int i, int j) {
        int key = getKey(i, j);
        List<Integer> current = this.current;
        int index = 0;
        while (index < current.size() && key > current.get(index)) {
            index++;
        }
        if (index < current.size() && key == current.get(index)) {
            return; // Pair was already added
        }
        current.add(index, key);
    }

    /**
     * Swap the current and previous overlap key lists.
     * Clears the current list for reuse.
     */
    public void tick() {
        List<Integer> tmp = this.current;
        this.current = this.previous;
        this.previous = tmp;
        this.current.clear();
    }

    /**
     * Calculate the difference between current and previous overlap key lists.
     * Additions are keys present in the current list but not in the previous list.
     * Removals are keys present in the previous list but not in the current list.
     */
    public void getDiff(List<Integer> additions, List<Integer> removals) {
        List<Integer> a = this.current;
        List<Integer> b = this.previous;

        int al = a.size();
        int bl = b.size();
        int j = 0;

        for (int i = 0; i < al; i++) {
            boolean found = false;
            int keyA = a.get(i);
            while (j < bl && keyA > b.get(j)) {
                j++;
            }
            found = j < bl && keyA == b.get(j);

            if (!found) {
                unpackAndPush(additions, keyA);
            }
        }

        j = 0;

        for (int i = 0; i < bl; i++) {
            boolean found = false;
            int keyB = b.get(i);
            while (j < al && keyB > a.get(j)) {
                j++;
            }
            found = j < al && keyB == a.get(j);

            if (!found) {
                unpackAndPush(removals, keyB);
            }
        }
    }

    // Helper method to unpack and add two integers to a list
    //check this (changed)
    private void unpackAndPush(List<Integer> array, int key) {
        array.add((key & 0xffff0000) >> 16);
        array.add(key & 0x0000ffff);
    }
}
