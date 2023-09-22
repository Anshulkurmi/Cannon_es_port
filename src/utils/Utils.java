package utils;

import java.util.Map;
import java.util.HashMap ;

public class Utils {

    /**
     * Extend an options object with default values.
     *
     * @param options  The options object. May be null: in this case, a new object is created and returned.
     * @param defaults An object containing default values.
     * @return The modified options object.
     */
    public static Map<String, Object> defaults(Map<String, Object> options, Map<String, Object> defaults) {
        if (options == null) {
            options = new HashMap<>();
        }

        for (String key : defaults.keySet()) {
            if (!options.containsKey(key)) {
                options.put(key, defaults.get(key));
            }
        }

        return options;
    }
}

