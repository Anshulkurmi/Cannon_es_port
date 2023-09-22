//package utils;

// import java.util.ArrayList;
// import java.util.HashMap;
// import java.util.List;
// import java.util.Map;
// import java.util.function.Consumer;

// /**
//  * Base class for objects that dispatch events.
//  */
// public class EventTarget {
//     private Map<String, List<Consumer<Object>>> listeners;

//     /**
//      * Add an event listener.
//      *
//      * @param type     The event type.
//      * @param listener The event listener.
//      * @return The self object, for chainability.
//      */
//     public EventTarget addEventListener(String type, Consumer<Object> listener) {
//         if (listeners == null) {
//             listeners = new HashMap<>();
//         }
        
//         listeners.computeIfAbsent(type, k -> new ArrayList<>()).add(listener);
//         return this;
//     }

//     /**
//      * Check if an event listener is added.
//      *
//      * @param type     The event type.
//      * @param listener The event listener.
//      * @return True if the listener is added, false otherwise.
//      */
//     public boolean hasEventListener(String type, Consumer<Object> listener) {
//         if (listeners == null) {
//             return false;
//         }
//         List<Consumer<Object>> listenerList = listeners.get(type);
//         return listenerList != null && listenerList.contains(listener);
//     }

//     /**
//      * Remove an event listener.
//      *
//      * @param type     The event type.
//      * @param listener The event listener.
//      * @return The self object, for chainability.
//      */
//     public EventTarget removeEventListener(String type, Consumer<Object> listener) {
//         if (listeners != null) {
//             List<Consumer<Object>> listenerList = listeners.get(type);
//             if (listenerList != null) {
//                 listenerList.remove(listener);
//             }
//         }
//         return this;
//     }

//     /**
//      * Emit an event.
//      *
//      * @param type The event type.
//      * @param data The event data.
//      * @return The self object, for chainability.
//      */
//     public EventTarget dispatchEvent(String type, Object data) {
//         if (listeners != null) {
//             List<Consumer<Object>> listenerList = listeners.get(type);
//             if (listenerList != null) {
//                 for (Consumer<Object> listener : listenerList) {
//                     listener.accept(data);
//                 }
//             }
//         }
//         return this;
//     }
// }

package utils;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * Base class for objects that dispatches events.
 */

public class EventTarget {
    private Map<String, List<EventListener>> listeners;

    public EventTarget() {
        listeners = new HashMap<>();
    }
    /**
   * Add an event listener
   * @return The self object, for chainability.
   */
    public void addEventListener(String type, EventListener listener) {
        // Add an event listener for the specified event type
        listeners.computeIfAbsent(type, k -> new ArrayList<>()).add(listener);
    }

    public boolean hasEventListener(String type, EventListener listener) {
        // Check if the specified event listener is registered for the event type
        return listeners.containsKey(type) && listeners.get(type).contains(listener);
    }

    public boolean hasAnyEventListener(String type) {
        // Check if any event listener is registered for the event type
        return listeners.containsKey(type);
    }

    public void removeEventListener(String type, EventListener listener) {
        // Remove the specified event listener for the event type
        if (listeners.containsKey(type)) {
            listeners.get(type).remove(listener);
        }
    }

    public void dispatchEvent(Event event) {
        String type = event.getType();
        EventTarget target = event.getTarget();

        // Check if the target has event listeners for the specified type
        if (target.listeners.containsKey(type)) {
            List<EventListener> typeListeners = target.listeners.get(type);
            for (EventListener listener : typeListeners) {
                // Invoke the event listener's handleEvent method
                listener.handleEvent(event);

                // Propagate the event to the parent if it is an instance of EventTarget
                if (target instanceof EventTarget) {
                    ((EventTarget) target).dispatchEvent(event);
                }
            }
        }
    }

    public interface EventListener {
        void handleEvent(Event event);
    }
}

class Event {
    private final String type;
    private final EventTarget target;

    public Event(String type, EventTarget target) {
        this.type = type;
        this.target = target;
    }

    public String getType() {
        return type;
    }

    public EventTarget getTarget() {
        return target;
    }
}
