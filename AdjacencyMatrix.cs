using System;
using System.Linq;
using System.Collections.Generic;

using GraphLib.Model;

namespace GraphLib
{
    public class AdjacencyMatrix<T>: Graph<T>
    {
        /*
        Dicionario contendo os dicionarios de arestas que representão o grafo
        Key: Nodo do grafo
        Value: Dicionario de Arestas do nodo
        O objeto Aresta contem um os atributos nodo e peso
        Representando o nodo a qual aquela aresta esta conectada e o peso da mesma
        */
        Dictionary <Vertex<T>, Dictionary<Vertex<T>, Edge<T>>> adjMatrix;

        public AdjacencyMatrix (CompareToDelegate<T> compareToDelegate, bool isDirected=false):base(compareToDelegate, isDirected)
        {
            Clean();
        }

        public AdjacencyMatrix (List<Vertex<T>> vertices, List<Edge<T>> edges, CompareToDelegate<T> compareToDelegate, bool isDirected=false):base(vertices, edges, compareToDelegate, isDirected){}

        protected override void Clean()
        {
            adjMatrix = new Dictionary<Vertex<T>, Dictionary<Vertex<T>, Edge<T>>>();
        }

        /*
        Adiconar um vertice ao grafo
        E preciso informar o valor no novo vertice
        */
        public override void AddVertex(T u)
        {
            //Obtem a lista de chaves do dicionario
            //Referencia direta aos vertices do grafo
            List<Vertex<T>> keys = adjMatrix.Keys.ToList();
            //Se não existir um vertice cujo valor seja igual ao valor do novo nodo
            if(!keys.Exists(x => x.value.Equals(u)))
            {
                //Instanciamos um novo vertice com o valor desejado
                Vertex<T> U = new Vertex<T>(u, compareToDelegate);
                //Adicionamos o mesmo ao dicionario junto a sua lista de arestastas
                Dictionary<Vertex<T>, Edge<T>> rowU = new Dictionary<Vertex<T>, Edge<T>>();
                adjMatrix.Add(U, rowU);

                foreach (Vertex<T> vertex in adjMatrix.Keys)
                {
                    rowU.Add(vertex, null);
                    if (!adjMatrix[vertex].Keys.Contains(U))
                        adjMatrix[vertex].Add(U, null);
                }
            }
        }

        /*
        Remover um vertice do grafo
        E preciso informar o valor do vertice a ser removido
        */
        public void RemoveVertex(T u)
        {
            //Obtem a lista de chaves do dicionario
            //Referencia direta aos vertices do grafo
            List<Vertex<T>> keys = adjMatrix.Keys.ToList();
            //Se existe um vertice cujo valor e igual ao que estamos procurando
            if(keys.Exists(x => x.value.Equals(u)))
            {
                //Encontramos o vertice que desejamos remover
                Vertex<T> vertex = keys.Find(x => x.value.Equals(u));
                //Para cada linha de adjacencia dos vertices do grafo
                foreach (Dictionary<Vertex<T>, Edge<T> > adjacencyRow in adjMatrix.Values)
                {
                    //Removemos a aresta
                    adjacencyRow.Remove(vertex);
                }
                //Removemos o vertice da lista de nodos do grafo
                adjMatrix.Remove(vertex);
            }
        }

        /*
        Adiciona uma aresta entre dois vertices pré-existentes
        E preciso inforvar o valor do vertice de origem U e o vertice de destino V
        Juntamente com o valor do peso da aresta
        Caso o grafo não seja direcionado esta função tera o efeito de
        AddEdge (u, v); AddEdge(v, u);
        Exerto caso u == v
        */
        public override void AddEdge(T u, T v, float weight=1)
        {
            //Obtem a lista de chaves do dicionario
            //Referencia direta aos vertices do grafo
            List<Vertex<T>> keys = adjMatrix.Keys.ToList();
            //Se existe um vertice cujo valor e igual a u, e um vertice cujo valor e igual a v
            if (keys.Exists(x => x.value.Equals(u)) && keys.Exists(x => x.value.Equals(v)))
            {
                //Encontramos o vertice U
                Vertex<T> U = keys.Find(x => x.value.Equals(u));
                //Encontramos o vertice V
                Vertex<T> V = keys.Find(x => x.value.Equals(v));

                //Instanciamos uma aresta entre U e V com o peso passado por paramentro
                Edge<T> UtoV = new Edge<T>(U, V, weight);
                //Adicionamos a nova aresta a posição de adjacencia de U a V
                adjMatrix[U][V] = UtoV;

                //Se o grafo não for direcionado e o U e V não forem o mesmo vertice
                if(!isDirected && !U.Equals(V))
                {
                    //Instanciamos uma aresta entre V e U com o peso passado por paramentro
                    Edge<T> VtoU = new Edge<T>(V, U, weight);
                    //Adicionamos a nova aresta a posição de adjacencia de V a U
                    adjMatrix[V][U] = VtoU;
                }
            }
        }

        /*
        Remove uma aresta entre dois vertices pré-existentes de peso weight
        E preciso inforvar o valor do vertice de origem U e o vertice de destino V
        Juntamente com o valor do peso da aresta
        */
        public void RemoveEdge(T u, T v)
        {
            //Obtem a lista de chaves do dicionario
            //Referencia direta aos vertices do grafo
            List<Vertex<T>> keys = adjMatrix.Keys.ToList();
            //Se existe um vertice cujo valor e igual a u, e um vertice cujo valor e igual a v
            if (keys.Exists(x => x.value.Equals(u)) && keys.Exists(x => x.value.Equals(v)))
            {
                //Encontramos o vertice U
                Vertex<T> U = keys.Find(x => x.value.Equals(u));
                //Encontramos o vertice U
                Vertex<T> V = keys.Find(x => x.value.Equals(v));
                //Removemos a referencia da aresta que conecta U a V
                adjMatrix[U][V] = null;
                //Se o grafo não for direcionado
                if (!isDirected)
                {
                    //Removemos a referencia da aresta que conecta V a U
                    adjMatrix[V][U] = null;
                }
            }
        }

        /*
        Calcula o grau de entrada de um vertice u
        O grau de entrada de um vertice e representado pelo
        numero de arestas que incidem sobre ele
        */
        public int EntryDegree(T u)
        {
            Vertex<T> U = adjMatrix.Keys.ToList().Find(x => x.value.Equals(u));
            int entry = 0;
            //Para cada lista de adjacencia do grafo
            foreach (Dictionary<Vertex<T>, Edge<T>> adjacentRow in adjMatrix.Values)
            {
                //Somamos ao entry o numero de arestas que possuem u como destino
                if(adjacentRow[U] != null)
                    ++entry;
            }
            return entry;
        }

        /*
        Calcula o grau de saida de um vertice u
        O grau de saida de um vertice e representado pelo
        numero de arestas partem dele
        */
        public int OutputDegree(T u)
        {
            //Obtem a lista de chaves do dicionario
            //Referencia direta aos vertices do grafo
            List< Vertex<T> > keys = adjMatrix.Keys.ToList();
            //Encontramos o vertice U
            Vertex<T> U = keys.Find(x => x.value.Equals(u));
            //Retornamos o numero de arestas que partem de U e
            //não são nulas
            return adjMatrix[U].Values.ToList().FindAll(x => x != null).Count();
        }

        /*
        Retorna uma lista de valores de vertices que são fontes do grafo
        Uma fonte e um vertice que possui o grau de entrada igual a 0
        */
        public List<T> Sources()
        {
            List<T> sources = new List<T>();
            //Para cada vertice do grafo
            foreach (Vertex<T> vertex in adjMatrix.Keys)
            {
                //Caso o grau de entrada o vertice seja 0
                if (EntryDegree(vertex.value) == 0)
                {
                    //Adicionamos o vertice a lista de fontes
                    sources.Add(vertex.value);
                }
            }
            return sources;
        }

        /*
        Retorna uma lista de valores de vertices que são sumidouros do grafo
        Um sumidouro e um vertice que possui o grau de saida igual a 0
        */
        public List<T> Drain()
        {
            List<T> drain = new List<T>();
            //Para cada vertice do grafo
            foreach (Vertex<T> vertex in adjMatrix.Keys)
            {
                //Caso o grau de saida o vertice seja 0
                if (OutputDegree(vertex.value) == 0)
                {
                    //Adicionamos o vertice a lista de sumidouros
                    drain.Add(vertex.value);
                }
            }
            return drain;
        }

        /*
        Busca em largura apartir de um vertice s
        */
        public void BFS(T s)
        {
            //Obtem a lista de chaves do dicionario
            //Referencia direta aos vertices do grafo
            List<Vertex<T> > keys = adjMatrix.Keys.ToList();
            //Encontramos o vertice S
            Vertex<T> S = keys.Find(x => x.value.Equals(s));
            //Para cada vertice do grafo
            foreach (Vertex<T> U in keys)
            {
                //Colocamos as variaveis internas do vertice em seus valores padrão
                //A cor braca sinaliza que o vertice ainda não foi visitado
                U.color = "WHITE";
                U.distance = float.PositiveInfinity;
                U.f = float.PositiveInfinity;
                //pi e a referencia ao vertice 'pai' visitado anteriormente
                U.pi = null;
            }
            
            //A busca inicia em S
            //A cor cinza sinaliza que os 'filhos' do vertice estão sendo visitados
            S.color = "GRAY";
            //A distancia do primeiro vertice é 0
            S.distance = 0;
            //O vertice de origem da busca não possui 'pai'
            S.pi = null;
            //Inicializamos a fila Q de vertices a serem visitado
            Queue<Vertex<T>> Q = new Queue<Vertex<T>>();
            //Enfileiramos S
            Q.Enqueue(S);
            //Enquanto houver vertices na fila Q
            while (Q.Count != 0)
            {
                //Desenfileiramos o proximo vertice em U
                Vertex<T> U = Q.Dequeue();
                //Para casa aresta partindo de U não nula
                foreach (Edge<T> edge in adjMatrix[U].Values.ToList().FindAll(x => x != null))
                {
                    //Obtemos o vertice V de destino da aresta
                    Vertex<T> V = edge.to;
                    //Caso o vertice ainda não tenha sido visitado
                    if (V.color == "WHITE")
                    {
                        //Sinalizamos que os 'filhos' do vertice V serao visitados
                        V.color = "GRAY";
                        //A distancia de V ate S, e a distancia do 'pai' de V, vulgo U mais 1 
                        V.distance = U.distance + 1;
                        //O pai de V é U
                        V.pi = U;
                        //Enfileiramos V
                        Q.Enqueue(V);
                    }
                }
                //Após visitar todos os vertices 'filhos' de U
                //Sinalizamos que U já foi totalmente visitado
                U.color = "BLACK";
            }
        }

        /*
        Busca em profundidade
        */
        public void DFS()
        {
            //Para cada vertice do grafo
            foreach (Vertex<T> U in adjMatrix.Keys)
            {
                //Colocamos as variaveis internas do vertice em seus valores padrão
                //A cor braca sinaliza que o vertice ainda não foi visitado
                U.color = "WHITE";
                U.distance = float.PositiveInfinity;
                U.f = float.PositiveInfinity;
                //pi e a referencia ao vertice 'pai' visitado anteriormente
                U.pi = null;
            }
            //Time representa os passos dados
            int time = 0;
            //Para cada vertice do grafo
            foreach (Vertex<T> U in adjMatrix.Keys)
            {
                //Caso o vertice ainda não tenha sido visitado
                if (U.color == "WHITE")
                {
                    //Visitamos o vertice U
                    DFSVisit(U, ref time);
                }
            }
        }

        /*
        Etapa recursica do algoritimo de DFS
        Visita ao vertice U
        */
        private void DFSVisit(Vertex<T> U, ref int time)
        {
            //Para casa visita contamos um passo
            time += 1;
            //A distancia do vertice U e a quantia de passos dados
            U.distance = time;
            //Sinalizamos que os 'filhos' do vertice V serao visitados
            U.color = "GRAY";
            //Para casa aresta partindo de U
            foreach (Edge<T> edge in adjMatrix[U].Values.ToList().FindAll(x => x != null))
            {
                //Obtemos o vertice V de destino da aresta
                Vertex<T> V = edge.to;
                if (V.color == "WHITE")
                {
                    //O pai de V é U
                    V.pi = U;
                    //Visitamos o vertice V
                    DFSVisit(V, ref time);
                }
            }
            //Após visitar todos os vertices 'filhos' de U
            //Sinalizamos que U já foi totalmente visitado
            U.color = "BLACK";
            //Ao fim de visitar os 'filhos' do vertice contamos um passo
            time += 1;
            U.f = time;
        }

        public override List<Edge<T>> Adjacent(T u)
        {
            //Obtem a lista de chaves do dicionario
            //Referencia direta aos vertices do grafo
            List<Vertex<T> > keys = adjMatrix.Keys.ToList();
            //Encontramos o vertice S
            Vertex<T> U = keys.Find(x => x.value.Equals(u));

            return Adjacent(U);
        }

        public override List<Edge<T>> Adjacent(Vertex<T> U)
        {
            return adjMatrix[U].Values.ToList().FindAll(x => x != null);
        }

        /*
        Ordenar o dicionario apartir da distancia ate a origem
        */
        public void Sort()
        {
            foreach (Vertex<T> vertex in adjMatrix.Keys)
            {
                Dictionary<Vertex<T>, Edge<T>> adjacencyRow = adjMatrix[vertex];
                adjMatrix[vertex] = adjacencyRow.OrderBy(x => x.Key.distance).ToDictionary(pair => pair.Key, pair => pair.Value);
            }
            adjMatrix = adjMatrix.OrderBy(x => x.Key.distance).ToDictionary(pair => pair.Key, pair => pair.Value);
        }

        /*
        Retorna a lista de arestas do grafo
        */
        public override List<Edge<T>> Edges()
        {
            List<Edge<T>> edges = new List<Edge<T>>();
            //Para cada linha de adjacencia
            foreach (Dictionary<Vertex<T>, Edge<T>> adjacencyRow in adjMatrix.Values)
            {
                //Para casa aresta do vertice
                foreach (Edge<T> edge in adjacencyRow.Values)
                {
                    //Se a aresta não for nula
                    if (edge != null)
                    {
                        //Adicionamos a aresta a lista
                        edges.Add(edge);
                    }
                }
            }
            //Ordenamos a lista apartir dos pesos da aresta
            EdgeComparer<T> comparer = new Model.EdgeComparer<T>();
            edges.Sort(comparer);
            return edges;
        }

        /*
        Retorna a lista de vertices do grafo
        */
        public override List<Vertex<T>> Vertices()
        {
            return adjMatrix.Keys.ToList();
        }

        /*
        Define a converção implicita entre uma lista de adjacencia
        e uma matriz de adjacecia
        */
        public static implicit operator AdjacencyMatrix<T> (AdjacencyList<T> list)
        {
            return new AdjacencyMatrix<T>(list.Vertices(), list.Edges(), list.CompareTo(), list.isDirected);
        }

        public override string ToString()
        {
            string toString = "\t";
            foreach (Vertex<T> vertex in adjMatrix.Keys)
            {
                toString += "|" + vertex.ToString() + "\t";
            }
            toString += "\n";

            foreach (Vertex<T> vertex in adjMatrix.Keys)
            {
                toString += vertex.ToString() + "\t";

                Dictionary<Vertex<T>, Edge<T>> adjacencyRow = adjMatrix[vertex];
                foreach (Edge<T> adjacencyCol in adjacencyRow.Values)
                {
                    if (adjacencyCol != null)
                    {
                        toString += "|" + adjacencyCol.weight.ToString() + "\t";
                    }
                    else
                    {
                        toString += "|    \t";
                    }
                }
                toString += "\n";
            }
            return toString;
        }
    }
}