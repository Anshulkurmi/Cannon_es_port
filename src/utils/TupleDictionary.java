package utils;

import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;

/**
 * TupleDictionary
 */
public class TupleDictionary {
    private Map<String, Object> data;
    private List<String> keys;

    public TupleDictionary() {
        data = new HashMap<>();
        keys = new LinkedList<>();
    }

    /**
     * get 
     */
    public Object get(int i, int j) {
        String key = this.getKey(i, j);
        return this.data.get(key);
    }

    /**
     * set
     */
    public void set(int i, int j, Object value) {
        String key = this.getKey(i, j);

        // Check if key already exists
        if (!this.data.containsKey(key)) {
            keys.add(key);
        }

        this.data.put(key, value);
    }

    /**
     * delete
     */
    public void delete(int i, int j) {
        String key = this.getKey(i, j);
        if (this.keys.contains(key)) {
            this.keys.remove(key);
        }
        this.data.remove(key);
    }

    /**
     * reset
     */
    public void reset() {
        for (String key : this.keys) {
            this.data.remove(key);
        }
        this.keys.clear();
    }

    private String getKey(int i, int j) {
        return (i < j) ? i + "-" + j : j + "-" + i;
    }

    
}

