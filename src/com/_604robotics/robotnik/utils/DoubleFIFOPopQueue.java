package com._604robotics.robotnik.utils;

import java.util.ArrayDeque;
import java.util.Iterator;

public class DoubleFIFOPopQueue
{
    private ArrayDeque<Double> content;
    public final int maxsize;

    public DoubleFIFOPopQueue(int maxsize)
    {
        this.content=new ArrayDeque<>(maxsize);
        this.maxsize=maxsize;
    }
    public int size()
    {
        return content.size();
    }

    public boolean isEmpty()
    {
        return content.isEmpty();
    }
    
    public boolean isFull()
    {
        return content.size()==this.maxsize;
    }
    
    public void remove()
    {
        content.removeLast();
    }

    public void add(double element)
    {
        if (this.size()<=this.maxsize)
        {
            content.addFirst(element);
        }
        else
        {
            remove();
            content.addFirst(element);
        }
    }
    
    public void flush()
    {
        while (!this.isFull())
        {
            add(0);
        }
    }
    
    public double currentAverage()
    {
        double sum=0;
        int elementCount=0;
        Iterator<Double> countloop=this.content.iterator();
        while(countloop.hasNext())
        {
            Double current=countloop.next();
            if (current!=null) {
            	sum+=current;
            	elementCount++;
            }
        }
        return sum/elementCount;
    }
    
    public Object[] toArray()
    {
        return this.content.toArray();
    }

}
