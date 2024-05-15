package events;

import objects.Body;

public class RemoveBodyEvent  extends Event{
    protected String type;
    public Body body;

    // Constructors, getters, and setters as needed

    public RemoveBodyEvent(String type, Body body) {
    	super(type,body);
        this.type = type;
        this.body = body;
    }

    // Getter and Setter methods
    // ...
}

