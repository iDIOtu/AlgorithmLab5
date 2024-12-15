// Graph.cs
using System;
using System.Collections.Generic;

public class Graph
{
    public List<Node> Nodes { get; set; } = new List<Node>();
    public List<Edge> Edges { get; set; } = new List<Edge>();

    public void AddNode(Node node)
    {
        Nodes.Add(node);
    }

    public void RemoveNode(Node node)
    {
        Nodes.Remove(node);
        Edges.RemoveAll(e => e.Start == node || e.End == node);
    }

    public void AddEdge(Node start, Node end, int weight)
    {
        Edges.Add(new Edge(start, end, weight));
    }

    public void RemoveEdge(Edge edge)
    {
        Edges.Remove(edge);
    }
}

public class Node
{
    public string Name { get; set; }
}

public class Edge
{
    public Node Start { get; set; }
    public Node End { get; set; }
    public int Weight { get; set; }

    public Edge(Node start, Node end, int weight)
    {
        Start = start;
        End = end;
        Weight = weight;
    }
}
