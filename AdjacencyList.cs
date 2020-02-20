using System;
using System.Linq;
using System.Collections.Generic;

using GraphLib.Model;

namespace GraphLib
{
    public class AdjacencyList<T>: Graph<T>
    {
        /*
        Dicionario contendo as listas encadeadas que representão o grafo
        Key: Nodo do grafo
        Value: Lista de Arestas do nodo
        O objeto Aresta contem um os atributos nodo e peso
        Representando o nodo a qual aquela aresta esta conectada e o peso da mesma
        */
        Dictionary <Vertex<T>, List<Edge<T>>> adjList;

        /*
        Instanciar um novo grafo por lista de adjacencia vaziu
        Exige apenas a informação de se o grafo sera ou não direcionado e a sua função de comparação
        */
        public AdjacencyList(CompareToDelegate<T> compareToDelegate, bool isDirected=false) : base(compareToDelegate, isDirected)
        {
            adjList = new Dictionary<Vertex<T>, List<Edge<T>>>();
        }

        public AdjacencyList (List<Vertex<T>> vertices, List<Edge<T>> edges, CompareToDelegate<T> compareToDelegate, bool isDirected=false):base(vertices, edges, compareToDelegate, isDirected){}

        protected override void Clean()
        {
            adjList = new Dictionary<Vertex<T>, List<Edge<T>>>();
        }

        /*
        Adiconar um vertice ao grafo
        E preciso informar o valor no novo vertice
        */
        public override void AddVertex(T u)
        {
            //Obtem a lista de chaves do dicionario
            //Referencia direta aos vertices do grafo
            List<Vertex<T>> keys = adjList.Keys.ToList();
            //Se não existir um vertice cujo valor seja igual ao valor do novo nodo
            if(!keys.Exists(x => x.value.Equals(u)))
            {
                //Instanciamos um novo vertice com o valor desejado
                Vertex<T> U = new Vertex<T>(u, compareToDelegate);
                //Adicionamos o mesmo ao dicionario junto a sua lista de arestastas
                adjList.Add(U, new List<Edge<T>>());
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
            List<Vertex<T>> keys = adjList.Keys.ToList();
            //Se existe um vertice cujo valor e igual ao que estamos procurando
            if(keys.Exists(x => x.value.Equals(u)))
            {
                //Encontramos o vertice que desejamos remover
                Vertex<T> vertex = keys.Find(x => x.value.Equals(u));
                //Para cada lista de adjacencia dos vertices do grafo
                foreach (List< Edge<T> > edge in adjList.Values)
                {
                    //Procuramos uma aresta cujo nodo seja o vertice que queremos remover
                    Edge<T> edgeToRemove = edge.Find(x => x.to.Equals(vertex));
                    //Removemos a aresta
                    edge.Remove(edgeToRemove);
                }
                //Removemos o vertice da lista de nodos do grafo
                adjList.Remove(vertex);
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
            List<Vertex<T>> keys = adjList.Keys.ToList();
            //Se existe um vertice cujo valor e igual a u, e um vertice cujo valor e igual a v
            if (keys.Exists(x => x.value.Equals(u)) && keys.Exists(x => x.value.Equals(v)))
            {
                //Encontramos o vertice U
                Vertex<T> U = keys.Find(x => x.value.Equals(u));
                //Encontramos o vertice V
                Vertex<T> V = keys.Find(x => x.value.Equals(v));

                //Instanciamos uma aresta entre U e V com o peso passado por paramentro
                Edge<T> UtoV = new Edge<T>(U, V, weight);
                //Adicionamos a nova aresta a lista de adjacencia de U
                if (!adjList[U].Contains(UtoV))
                    adjList[U].Add(UtoV);

                //Se o grafo não for direcionado e o U e V não forem o mesmo vertice
                if(!isDirected && !U.Equals(V))
                {
                    //Instanciamos uma aresta entre V e U com o peso passado por paramentro
                    Edge<T> VtoU = new Edge<T>(V, U, weight);
                    //Adicionamos a nova aresta a lista de adjacencia de V
                    if (!adjList[V].Contains(VtoU))
                        adjList[V].Add(VtoU);
                }
            }
        }

        /*
        Remove uma aresta entre dois vertices pré-existentes de peso weight
        E preciso inforvar o valor do vertice de origem U e o vertice de destino V
        Juntamente com o valor do peso da aresta
        */
        public void RemoveEdge(T u, T v, float weight=1)
        {
            //Obtem a lista de chaves do dicionario
            //Referencia direta aos vertices do grafo
            List<Vertex<T>> keys = adjList.Keys.ToList();
            //Se existe um vertice cujo valor e igual a u, e um vertice cujo valor e igual a v
            if (keys.Exists(x => x.value.Equals(u)) && keys.Exists(x => x.value.Equals(v)))
            {
                //Encontramos o vertice U
                Vertex<T> U = keys.Find(x => x.value.Equals(u));
                //Encontramos o vertice U
                Vertex<T> V = keys.Find(x => x.value.Equals(v));
                //Removemos da lista de adjacencia de U o vertice que possui destino V e peso weight
                adjList[U].Remove(adjList[U].Find(x => (x.to.Equals(V) && x.weight == weight)));
                //Se o grafo não for direcionado
                if (!isDirected)
                {
                    //Removemos da lista de adjacencia de V o vertice que possui destino U e peso weight
                    adjList[V].Remove(adjList[V].Find(x => (x.to.Equals(U) && x.weight == weight)));
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
            int entry = 0;
            //Para cada lista de adjacencia do grafo
            foreach (List<Edge<T>> adjacentList in adjList.Values)
            {
                //Somamos ao entry o numero de arestas que possuem u como destino
                entry += adjacentList.FindAll(x => x.to.value.Equals(u)).Count;
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
            List< Vertex<T> > keys = adjList.Keys.ToList();
            //Encontramos o vertice U
            Vertex<T> U = keys.Find(x => x.value.Equals(u));
            //Retornamos o numero de arestas que partem de U
            return adjList[U].Count();
        }

        /*
        Retorna uma lista de valores de vertices que são fontes do grafo
        Uma fonte e um vertice que possui o grau de entrada igual a 0
        */
        public List<T> Sources()
        {
            List<T> sources = new List<T>();
            //Para cada vertice do grafo
            foreach (Vertex<T> vertex in adjList.Keys)
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
            foreach (Vertex<T> vertex in adjList.Keys)
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
            List<Vertex<T> > keys = adjList.Keys.ToList();
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
                //Para casa aresta partindo de U
                foreach (Edge<T> edge in adjList[U])
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
            foreach (Vertex<T> U in adjList.Keys)
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
            foreach (Vertex<T> U in adjList.Keys)
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
            foreach (Edge<T> edge in adjList[U])
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
            List<Vertex<T> > keys = adjList.Keys.ToList();
            //Encontramos o vertice S
            Vertex<T> U = keys.Find(x => x.value.Equals(u));

            return Adjacent(U);
        }

        public override List<Edge<T>> Adjacent(Vertex<T> U)
        {
            return adjList[U];
        }

        /*
        Ordenar o dicionario apartir da distancia ate a origem
        */
        public void Sort()
        {
            adjList = adjList.OrderBy(x => x.Key.distance).ToDictionary(pair => pair.Key, pair => pair.Value);
        }

        /*
        Retorna a lista de arestas do grafo
        */
        public override List<Edge<T>> Edges()
        {
            List<Edge<T>> edges = new List<Edge<T>>();
            int lenNodo = adjList.Values.Count;
            //Para cada lista de adjacencia
            for (int i = 0; i < lenNodo; i++)
            {
                List< Edge<T> > arestas = adjList.Values.ToList()[i];
                int lenAresta = arestas.Count;
                //Para cada aresta
                for (int j = 0; j < lenAresta; j++)
                {
                    //Adicionamos a aresta a lista
                    edges.Add(arestas[j]);
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
            return adjList.Keys.ToList();
        }

        public static implicit operator AdjacencyList<T> (AdjacencyMatrix<T> m)
        {
            return new AdjacencyList<T>(m.Vertices(), m.Edges(), m.CompareTo(), m.isDirected);
        }

        /*
        Representação em string da lista de adjacencia
        */
        public override string ToString()
        {
            string result = "";
            int lenNodos = adjList.Count;

            foreach (var nodo in adjList.Keys)
            {
                result += "[" + nodo.ToString() + "]";

                foreach (var aresta in adjList[nodo])
                {
                    result += " -> " + aresta.ToString();
                }
                result += "\n";
            }

            return result;
        }
    }
}