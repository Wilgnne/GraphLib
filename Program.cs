using System;

namespace GraphLib
{
    class Program
    {
        static void Main(string[] args)
        {
            AdjacencyList<char> l = new AdjacencyList<char>((x, y) => x.CompareTo(y), false);

            /*
            l.AddVertex('a');
            l.AddVertex('b');
            l.AddVertex('c');
            l.AddVertex('d');
            l.AddVertex('e');
            l.AddVertex('f');

            l.AddEdge('a', 'b', 10);
            l.AddEdge('b', 'c', 1);
            l.AddEdge('c', 'e', 3);
            l.AddEdge('e', 'd', -10);
            l.AddEdge('d', 'b', 4);
            l.AddEdge('e', 'f', 22);

            l.RemoveEdge('e', 'd', -10);
            l.AddEdge('e', 'd', 2);

            l.AddVertex('g');
            l.AddEdge('d', 'g', 14);

            l.AddEdge('b', 'd', 10);

            l.Edges().ForEach(x => Console.WriteLine(x));

            Console.WriteLine(l);

            Console.WriteLine(TreeAlgorithms<char>.Kruskal(l));
            */

            l.AddVertex('a');
            l.AddVertex('b');
            l.AddVertex('c');
            l.AddVertex('d');
            l.AddVertex('e');
            l.AddVertex('f');
            l.AddVertex('g');
            l.AddVertex('h');
            l.AddVertex('i');

            l.AddEdge('a', 'b', 4);
            l.AddEdge('a', 'h', 8);

            l.AddEdge('b', 'c', 8);
            l.AddEdge('b', 'h', 11);

            l.AddEdge('c', 'd', 7);
            l.AddEdge('c', 'f', 4);
            l.AddEdge('c', 'i', 2);

            l.AddEdge('d', 'e', 9);
            l.AddEdge('d', 'f', 14);

            l.AddEdge('e', 'f', 10);

            l.AddEdge('f', 'g', 2);

            l.AddEdge('g', 'h', 1);

            l.Edges().ForEach(x => Console.WriteLine(x));

            Console.WriteLine(l);

            Console.WriteLine(TreeAlgorithms<char>.Kruskal(l));

        }
    }
}
