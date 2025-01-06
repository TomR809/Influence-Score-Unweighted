internal class Edge
{
    public Node startNode { get; set; }  
    public Node endNode { get; set; }
    public int weight { get; set; }

    public Edge(Node node1, Node node2, int w)
    {
        startNode = node1;
        endNode = node2;
        weight = w;
    }
}