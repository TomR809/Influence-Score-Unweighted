internal class Node
{
    public char name { get;}
    public int g { get; set; } = int.MaxValue;
    public int f { get; set; } = int.MaxValue;
    public Node? parent { get; set; } = null; 

    public Node(char n)
    {
        name = n;
    }

    public void Reset()
    {
        g = int.MaxValue; // Set g to "infinity" (initial state)
        f = int.MaxValue; // Set f to "infinity" (initial state)
        parent = null;    // Clear parent
    }
}