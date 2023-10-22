using System.Collections.Generic;
using System.Runtime.InteropServices;

namespace CDT;

public class KDTree
{
    const int NumVerticesInLeaf = 32;
    enum NodeSplitDirection { X, Y }
    struct Node
    {
        public int child0, child1;
        public List<int> data;
        public Node() { child0 = 0; child1 = 0; data = new(); }
        public void SetChildren(int c1, int c2) { child0 = c1; child1 = c2; }
        public bool IsLeaf() => child0 == child1;
    }
    public KDTree(Vector2? min = null, Vector2? max = null)
    {
        m_min = min ?? new(double.NegativeInfinity, double.NegativeInfinity);
        m_max = max ?? new(double.PositiveInfinity, double.PositiveInfinity);
        m_root = AddNewNode();
    }
    public KDTree(List<Vector2> points)
    {
        m_min = m_max = points[0];
        for (int i = 1; i < points.Count; i++)
        { m_min = Vector2.Min(m_min, points[i]); m_max = Vector2.Max(m_max, points[i]); }
        m_root = AddNewNode();
        for (int i = 0; i < points.Count; i++) Insert(i, points);
    }
    public int Size => m_size;
    public void Insert(int iPoint, List<Vector2> points)
    {
        ++m_size;
        Vector2 pos = points[iPoint];
        while (!IsInsideBox(pos, m_min, m_max)) ExtendTree(pos);
        int node = m_root; Vector2 min = m_min, max = m_max; NodeSplitDirection dir = m_rootDir, newDir = 0;
        double mid; Vector2 newMin, newMax;
        while (true)
        {
            var s_nodes = CollectionsMarshal.AsSpan(m_nodes);
            if (s_nodes[node].IsLeaf())
            {
                List<int> pd = s_nodes[node].data;
                if (pd.Count < NumVerticesInLeaf) { pd.Add(iPoint); return; }
                if (!m_isRootBoxInitialized) { InitializeRootBox(points); min = m_min; max = m_max; }
                CalcSplitInfo(min, max, dir, out mid, out newDir, out newMin, out newMax);
                int c1 = AddNewNode(), c2 = AddNewNode(); Node n = m_nodes[node];
                s_nodes[node].SetChildren(c1, c2);
                List<int> c1Data = m_nodes[c1].data, c2Data = m_nodes[c2].data;
                for (int i = 0; i < n.data.Count; i++)
                {
                    if (!WhichChild(points[n.data[i]], mid, dir)) c1Data.Add(n.data[i]);
                    else c2Data.Add(n.data[i]);
                }
                s_nodes[node].data = new();
            }
            else CalcSplitInfo(min, max, dir,out mid, out newDir, out newMin, out newMax);
            bool iChild = WhichChild(points[iPoint], mid, dir);
            if (!iChild) max = newMax; else min = newMin;
            node = iChild ? m_nodes[node].child1 : m_nodes[node].child0;
            dir = newDir;
        }
    }
    public (Vector2, int) Nearest(Vector2 point, List<Vector2> points)
    {
        (Vector2, int) out_ = new(); double minDistSq = double.PositiveInfinity;
        m_tasksStack.Push(new(m_root, m_min, m_max, m_rootDir, minDistSq));
        while (m_tasksStack.Count > 0)
        {
            NearestTask t = m_tasksStack.Pop();
            if (t.distSq > minDistSq) continue;
            Node n = m_nodes[t.node];
            if (n.IsLeaf())
            {
                for (int i = 0; i < n.data.Count; i++)
                {
                    Vector2 p = points[n.data[i]]; double distSq = Vector2.DistanceSquared(point, p);
                    if (distSq < minDistSq) { minDistSq = distSq; out_ = (p, n.data[i]); }
                }
            }
            else
            {
                CalcSplitInfo(t.min, t.max, t.dir, out double mid, out var newDir, out Vector2 newMin, out Vector2 newMax);
                double distToMid = t.dir == NodeSplitDirection.X ? point.X - mid : point.Y - mid,
                    toMidSq = distToMid * distToMid;
                if (!WhichChild(point, mid, t.dir))
                {
                    m_tasksStack.Push(new(n.child1, newMin, t.max, newDir, toMidSq));
                    m_tasksStack.Push(new(n.child0, t.min, newMax, newDir, toMidSq));
                }
                else
                {
                    m_tasksStack.Push(new(n.child0, t.min, newMax, newDir, toMidSq));
                    m_tasksStack.Push(new(n.child1, newMin, t.max, newDir, toMidSq));
                }
            }
        }
        return out_;
    }
    int AddNewNode() { m_nodes.Add(new()); return m_nodes.Count - 1; }
    bool WhichChild(Vector2 point, double split, NodeSplitDirection dir) =>
        dir == NodeSplitDirection.X ? point.X > split : point.Y > split;
    static void CalcSplitInfo(Vector2 min, Vector2 max, NodeSplitDirection dir, out double midOut,
        out NodeSplitDirection newDirOut, out Vector2 newMinOut, out Vector2 newMaxOut)
    {
        midOut = 0; newMaxOut = max; newMinOut = min; newDirOut = 0;
        switch (dir)
        {
            case NodeSplitDirection.X:
                midOut = (min.X + max.X) / 2;
                newDirOut = NodeSplitDirection.Y; newMinOut.X = newMaxOut.X = midOut;
                break;
            case NodeSplitDirection.Y:
                midOut = (min.Y + max.Y) / 2;
                newDirOut = NodeSplitDirection.X; newMinOut.Y = newMaxOut.Y = midOut;
                break;
        }
    }
    static bool IsInsideBox(Vector2 p, Vector2 min, Vector2 max) =>
        p.X >= min.X && p.X <= max.X && p.Y >= min.Y && p.Y <= max.Y;
    void ExtendTree(Vector2 point)
    {
        int newRoot = AddNewNode(), newLeaf = AddNewNode();
        switch (m_rootDir)
        {
            case NodeSplitDirection.X:
                m_rootDir = NodeSplitDirection.Y;
                if (point.Y < m_min.Y) m_nodes[newRoot].SetChildren(newLeaf, m_root);
                else m_nodes[newRoot].SetChildren(m_root, newLeaf);
                if (point.Y < m_min.Y) m_min.Y -= m_max.Y - m_min.Y;
                else if (point.Y > m_max.Y) m_max.Y += m_max.Y - m_min.Y;
                break;
            case NodeSplitDirection.Y:
                m_rootDir = NodeSplitDirection.X;
                if (point.X < m_min.X) m_nodes[newRoot].SetChildren(newLeaf, m_root);
                else m_nodes[newRoot].SetChildren(m_root, newLeaf);
                if (point.X < m_min.X) m_min.X -= m_max.X - m_min.X;
                else if (point.X > m_max.X) m_max.X += m_max.X - m_min.X;
                break;
            default:
                break;
        }
        m_root = newRoot;
    }
    void InitializeRootBox(List<Vector2> points)
    {
        m_min = m_max = points[0];
        for (int i = 1; i < m_nodes[m_root].data.Count; i++)
        {
            Vector2 p = points[m_nodes[m_root].data[i]];
            m_min = Vector2.Min(m_min, p);
            m_max = Vector2.Max(m_max, p);
        }
        double padding = 1;
        if (m_min.X == m_max.X) { m_min.X -= padding; m_max.X += padding; }
        if (m_min.Y == m_max.Y) { m_min.Y -= padding; m_max.Y += padding; }
        m_isRootBoxInitialized = true;
    }
    int m_root;
    List<Node> m_nodes = new();
    NodeSplitDirection m_rootDir;
    Vector2 m_min, m_max;
    int m_size;
    bool m_isRootBoxInitialized;
    struct NearestTask
    {
        public int node; public Vector2 min, max; public NodeSplitDirection dir; public double distSq;
        public NearestTask(int node, Vector2 min, Vector2 max, NodeSplitDirection dir, double distSq)
        { this.node = node; this.min = min; this.max = max; this.dir = dir; this.distSq = distSq; }
    }
    Stack<NearestTask> m_tasksStack = new();
}
