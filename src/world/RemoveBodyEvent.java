package world;

import objects.Body;

public class RemoveBodyEvent {
    private String type;
    public Body body;

    // Constructors, getters, and setters as needed

    public RemoveBodyEvent(String type, Body body) {
        this.type = type;
        this.body = body;
    }

    // Getter and Setter methods
    // ...
}

