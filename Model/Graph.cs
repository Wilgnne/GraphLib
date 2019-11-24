using System;
using System.Collections.Generic;

namespace GraphLib.Model
{
    public abstract class Graph<T>
    {
        /*
        Indica se o grafo é ou não direcionado
        */
        public bool isDirected { get; }

        /*
        Função responsavel por informar se um nodo e maior, igual ou menor que outro
        */
        protected CompareToDelegate<T> compareToDelegate;

        public Graph (CompareToDelegate<T> compareToDelegate, bool isDirected=false)
        {
            this.compareToDelegate = compareToDelegate;
            this.isDirected = isDirected;
        }

        /*
        Instancia um novo grafo por lista de adjacencia
        Apartir de uma lista de vertices e uma lista de arestas
        E presiso informar se o grafo gerado sera ou não direcionado, e sua função de comparação
        */
        public Graph(List<Vertex<T>> vertices, List<Edge<T>> edges, CompareToDelegate<T> compareToDelegate, bool isDirected=false)
        {
            this.compareToDelegate = compareToDelegate;
            this.isDirected = isDirected;
            Clean();
            foreach (Vertex<T> vertex in vertices)
            {
                AddVertex(vertex.value);
            }

            foreach (Edge<T> edge in edges)
            {
                AddEdge(edge.from.value, edge.to.value, edge.weight);
            }
        }

        protected abstract void Clean();
        public abstract void AddVertex (T u);
        public abstract void AddEdge (T u, T v, float weight=1);
        public abstract List<Edge<T>> Adjacent (T u);
        public abstract List<Edge<T>> Adjacent (Vertex<T> U);
        public abstract List<Vertex<T>> Vertices ();

        public abstract List<Edge<T>> Edges();

        public CompareToDelegate<T> CompareTo()
        {
            return compareToDelegate;
        }
    }
}