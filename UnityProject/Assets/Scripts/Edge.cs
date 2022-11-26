using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Edge
{
    public int a;
    public int b;
    public int other;

    public float length = 0;
    public float volume = 0;
    public float stiffness = 0;

    public enum EdgeType
    {
        Traction,
        Bending
    }

    public EdgeType edgeType = EdgeType.Traction;

    public Edge(int x, int y, int other, float stiffness)
    {
        if (x < y)
        {
            a = x;
            b = y;
        }
        else
        {
            a = y;
            b = x;
        }
        this.other = other;
        this.stiffness = stiffness;
    }
}


public class EdgeEqualityComparer : IEqualityComparer<Edge>
{
    public bool Equals(Edge e1, Edge e2)
    {
        return e1.a == e2.a && e1.b == e2.b;
    }
    public int GetHashCode(Edge e)
    {
        int hcode = (e.a * 1000) + e.b;//Se calcula un número entero asociado a la arista
        return hcode.GetHashCode();
    }
}
