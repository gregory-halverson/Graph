/**
 * Created by Gregory on 2014-05-11.
 */
public class TestPQ
{
    public static void main(String [] args)
    {
        PriorityQueue<Integer> q = new PriorityQueue<Integer>();

        q.enqueue(34, 3.0);
        q.enqueue(34, 2.0);
        q.enqueue(56, 2.5);

        while (!q.empty())
            System.out.println(q.dequeue());
    }
}
