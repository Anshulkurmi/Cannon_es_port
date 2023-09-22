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
        String key = getKey(i, j);
        return data.get(key);
    }

    /**
     * set
     */
    public void set(int i, int j, Object value) {
        String key = getKey(i, j);

        // Check if key already exists
        if (!data.containsKey(key)) {
            keys.add(key);
        }

        data.put(key, value);
    }

    /**
     * delete
     */
    public void delete(int i, int j) {
        String key = getKey(i, j);
        if (keys.contains(key)) {
            keys.remove(key);
        }
        data.remove(key);
    }

    /**
     * reset
     */
    public void reset() {
        for (String key : keys) {
            data.remove(key);
        }
        keys.clear();
    }

    private String getKey(int i, int j) {
        return (i < j) ? i + "-" + j : j + "-" + i;
    }

    
}

