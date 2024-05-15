package events;

import objects.Body;

public class AddBodyEvent extends Event {
    public Body body;


    public AddBodyEvent(String type, Body body) {
    	super(type, body);
        this.type = type;
        this.body = body;
    }
}
