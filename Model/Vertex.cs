namespace GraphLib.Model
{
    public delegate int CompareToDelegate<T> (T u, T v);
    public class Vertex<T>
    {

        public T value;
        public CompareToDelegate<T> compareTo;
        public string color = "WHITE";
        public Vertex<T> pi = null;
        public float distance = float.PositiveInfinity;
        public float f = float.PositiveInfinity;
        public float key = float.PositiveInfinity;

        public Vertex(T value, CompareToDelegate<T> compareTo)
        {
            this.compareTo = compareTo;
            this.value = value;
        }

        public int CompareTo(Vertex<T> V)
        {
            return compareTo(value, V.value);
        }

        public override string ToString()
        {
            return value.ToString() + ":" + distance.ToString() + "/"+ f.ToString();
        }
    }   
}