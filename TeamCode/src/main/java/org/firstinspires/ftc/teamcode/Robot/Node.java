package org.firstinspires.ftc.teamcode.Robot;

import java.util.function.Supplier;

public class Node {
    private Runnable operation;
    public Node[] next;
    public int index=0;
    private Supplier<Boolean> condition;
    public  String name;

    public Node(String name) {
        this.name=name;
    }








    public void addConditions(Runnable operation , Supplier<Boolean> condition , Node[] next) {
        this.operation=operation;
        this.condition=condition;
        this.next=next;
    }

    public void run()
    {

        operation.run();

    }
    public boolean transition()
    {
        return condition.get();
    }
}