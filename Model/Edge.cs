using System;
using System.Collections.Generic;

namespace GraphLib.Model
{
    public class Edge<T>
    {
        public Vertex<T> from;
        public Vertex<T> to;
        public float weight;

        public Edge(Vertex<T> from, Vertex<T> to, float weight)
        {
            this.from = from;
            this.to = to;
            this.weight = weight;
        }

        public override string ToString()
        {
            return "(" + from.value.ToString() + ", " + to.value.ToString() + ", " + weight.ToString() +")";
        }

        public int CompareTo(Edge<T> V)
        {
            return from.CompareTo(V.from);
        }
    }

    class EdgeComparer<T>: IComparer<Edge<T>>
    {
        public int Compare (Edge<T> x, Edge<T> y)
        {
            if (x == null)
            {
                if (y == null)
                {
                    // Se x e nulo e y e nulo, entao
                    // Iguais
                    return 0;
                }
                else
                {
                    // Se x e nulo e y nao é, y
                    // e maior;
                    return -1;
                }
            }
            else
            {
                // Se x não e nulo ...
                if (y == null)
                {
                    // ... e y é nulo, x e maior;
                    return 1;
                }
                // Se nem x nem y sao nulos
                else
                {
                    // Se o peso de x e maior que o de y
                    if (x.weight > y.weight)
                    {
                        // x é o maior
                        return 1;
                    }
                    // Se o peso de x e menor que o de y
                    else if (x.weight < y.weight)
                    {
                        // y é o maior
                        return -1;
                    }
                    // Se os pesos forem iguais
                    else
                    {
                        EdgeEqualityComparer<T> comparer = new EdgeEqualityComparer<T>();
                        if(comparer.Equals(x, y))
                        {
                            return x.CompareTo(y);
                        }
                        else
                        {
                            int ft = x.from.CompareTo(y.to);
                            int tf = x.to.CompareTo(y.from);
                            if(ft == 1 && tf == -1)
                            {
                                return 1;
                            }
                            else if(ft == 1 && tf == 1)
                            {
                                return 1;
                            }
                            else if(ft == -1 && tf == 1)
                            {
                                return -1;
                            }
                            return -1;
                        }
                    }
                }
            }
        }
    }

    class EdgeEqualityComparer<T>: IEqualityComparer< Edge<T> >
    {
        public bool Equals(Edge<T> a1, Edge<T> a2)
        {
            if (a2 == null && a1 == null)
                return true;
            else if (a1 == null || a2 == null)
                return false;
            
            else if ((a1.from.Equals(a2.from) && a1.to.Equals(a2.to) && a1.weight.Equals(a2.weight)) || (a1.from.Equals(a2.to) && a1.to.Equals(a2.from) && a1.weight.Equals(a2.weight)))
            {
                return true;
            }
            else
            {
                return false;
            }
        }

        public int GetHashCode(Edge<T> a)
        {
            int hash = a.from.GetHashCode() ^ a.to.GetHashCode() ^ a.weight.GetHashCode();
            return hash;
        }
    }
}