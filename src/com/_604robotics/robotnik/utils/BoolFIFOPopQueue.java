package com._604robotics.robotnik.utils;

import java.util.ArrayDeque;
import java.util.Iterator;

public class BoolFIFOPopQueue
{
    private ArrayDeque<Boolean> content;
    public final int maxsize;
    public final double fraction;

    public BoolFIFOPopQueue(int maxsize, double fraction)
    {
        this.content=new ArrayDeque<>(maxsize);
        this.maxsize=maxsize;
        this.fraction=fraction;
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

    public void add(boolean element)
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
        while (this.currentFraction()!=0 || !this.isFull())
        {
            add(false);
        }
    }
    
    public double currentFraction()
    {
        int count=0;
        Iterator<Boolean> countloop=this.content.iterator();
        while(countloop.hasNext())
        {
            if ((boolean) countloop.next())
            {
                count++;
            }
        }
        return (double)count/(double)size();
    }
    
    public boolean passThreshold()
    {
        return this.currentFraction()>=this.fraction;
    }
    
    public Object[] toArray()
    {
        return this.content.toArray();
    }

}
