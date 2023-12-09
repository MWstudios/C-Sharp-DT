using System;
using System.Linq;
using System.Diagnostics;
using System.Collections.Generic;

namespace CDT;
public class CDT
{
    const int noNeighbor = int.MaxValue;
    const int invalidIndex = int.MaxValue;
    public List<Vector2> vertices = new(); public List<Triangle> triangles = new(); public HashSet<Edge> fixedEdges = new();
    public Dictionary<Edge, ushort> overlapCount = new(); public Dictionary<Edge, List<Edge>> pieceToOriginals = new();
    public CDT() { }
    /// <param name="vertexInsertionOrder">strategy used for ordering vertex insertions</param>
    public CDT(VertexInsertionOrder vertexInsertionOrder) { this.vertexInsertionOrder = vertexInsertionOrder; }
    /// <param name="vertexInsertionOrder">strategy used for ordering vertex insertions</param>
    /// <param name="intersectingEdgesStrategy">strategy for treating intersecting constraint edges</param>
    /// <param name="minDistToConstraintEdge">distance within which point is considered to be lying on a constraint edge. Used when adding constraints to the triangulation.</param>
    public CDT(VertexInsertionOrder vertexInsertionOrder, IntersectingConstraintEdges intersectingEdgesStrategy, double minDistToConstraintEdge)
    {
        this.vertexInsertionOrder = vertexInsertionOrder; this.intersectingEdgesStrategy = intersectingEdgesStrategy; this.minDistToConstraintEdge = minDistToConstraintEdge;
    }
    /// <param name="vertexInsertionOrder">strategy used for ordering vertex insertions</param>
    /// <param name="nearPtLocator">class providing locating near point for efficiently inserting new points</param>
    /// <param name="intersectingEdgesStrategy">strategy for treating intersecting constraint edges</param>
    /// <param name="minDistToConstraintEdge">distance within which point is considered to be lying on a constraint edge. Used when adding constraints to the triangulation.</param>
    public CDT(VertexInsertionOrder vertexInsertionOrder, KDTree nearPtLocator, IntersectingConstraintEdges intersectingEdgesStrategy, double minDistToConstraintEdge)
    {
        this.vertexInsertionOrder = vertexInsertionOrder; this.nearPtLocator = nearPtLocator;
        this.intersectingEdgesStrategy = intersectingEdgesStrategy; this.minDistToConstraintEdge = minDistToConstraintEdge;
    }
    public record struct DuplicatesInfo(List<int> mapping, List<int> duplicates);
    public static DuplicatesInfo FindDuplicates(ICollection<Vector2> vertices)
    {
        Dictionary<Vector2, int> uniqueVerts = new();
        DuplicatesInfo di = new(); int iOut = 0, iIn = -1;
        foreach (var first in vertices)
        {
            iIn++;
            bool isUnique = !uniqueVerts.ContainsKey(first);
            if (isUnique)
            {
                uniqueVerts[first] = iOut;
                di.mapping.Add(iOut++);
            }
            else
            {
                di.mapping.Add(iOut);
                di.duplicates.Add(iIn++);
            }
        }
        return di;
    }
    public static void RemoveDuplicates(List<int> vertices, List<int> duplicates) => vertices.RemoveAll(x => duplicates.Contains(x));
    public static Dictionary<Edge, List<int>> EdgeToSplitVertices(Dictionary<Edge, List<Edge>> edgeToPieces, List<Vector2> vertices)
    {
        Dictionary<Edge, List<int>> edgeToSplitVerts = new();
        foreach (var e2pIt in edgeToPieces)
        {
            Edge e = e2pIt.Key;
            Vector2 d = vertices[e.iV2] - vertices[e.iV1];
            bool isX = Math.Abs(d.X) >= Math.Abs(d.Y), isAscending = isX ? d.X >= 0 : d.Y >= 0;
            List<Edge> pieces = e2pIt.Value;
            List<(int, double)> splitVerts = new(pieces.Count + 1);
            foreach (var pieceIt in pieces)
            {
                int[] vv = new int[] { pieceIt.iV1, pieceIt.iV2 };
                for (int v = 0; v < 2; v++)
                {
                    double c = isX ? vertices[vv[v]].X : vertices[vv[v]].Y;
                    splitVerts.Add((vv[v], isAscending ? c : -c));
                }
            }
            splitVerts.Sort((a, b) => a.Item2.CompareTo(b.Item2));
            splitVerts = splitVerts.Distinct().ToList();
            Debug.Assert(splitVerts.Count > 2);
            (Edge, List<int>) val = (e, new(splitVerts.Count));
            foreach (var it in splitVerts.Skip(1).SkipLast(1)) val.Item2.Add(it.Item1);
            edgeToSplitVerts[val.Item1] = val.Item2;
        }
        return edgeToSplitVerts;
    }
    public void InsertVertices(Span<Vector2> vertices)
    {
        if (IsFinalized()) throw new ArgumentException("Triangulation was finalized with 'erase...' method. Inserting new vertices is not possible");
        bool isFirstTime = this.vertices.Count == 0;
        double max = double.PositiveInfinity;
        Box box = new() { min = new(max, max), max = new(max - max, -max) };
        if (this.vertices.Count == 0) { box = Box.EnvelopBox(vertices); AddSuperTriangle(box); }
        TryInitNearestPointLocator();
        int nExistingVerts = this.vertices.Count, nVerts = nExistingVerts + vertices.Length;
        triangles.EnsureCapacity(triangles.Count + 2 * nVerts);
        this.vertices.EnsureCapacity(nVerts); vertTris.EnsureCapacity(nVerts);
        foreach (var it in vertices) AddNewVertex(it, noNeighbor);
        switch (vertexInsertionOrder)
        {
            case VertexInsertionOrder.AsProvided: InsertVertices_AsProvided(nExistingVerts); break;
            case VertexInsertionOrder.Auto:
                if (isFirstTime) InsertVertices_KDTreeBFS(nExistingVerts, box.min, box.max);
                else InsertVertices_Randomized(nExistingVerts); break;
            default: break;
        }
    }
    public void InsertVertices(ICollection<Vector2> vertices)
    {
        if (IsFinalized()) throw new ArgumentException("Triangulation was finalized with 'erase...' method. Inserting new vertices is not possible");
        bool isFirstTime = this.vertices.Count == 0;
        double max = double.PositiveInfinity;
        Box box = new() { min = new(max, max), max = new(max - max, -max) };
        if (this.vertices.Count == 0) { box = Box.EnvelopBox(vertices); AddSuperTriangle(box); }
        TryInitNearestPointLocator();
        int nExistingVerts = this.vertices.Count, nVerts = nExistingVerts + vertices.Count;
        triangles.EnsureCapacity(triangles.Count + 2 * nVerts);
        this.vertices.EnsureCapacity(nVerts); vertTris.EnsureCapacity(nVerts);
        foreach (var it in vertices) AddNewVertex(it, noNeighbor);
        switch (vertexInsertionOrder)
        {
            case VertexInsertionOrder.AsProvided: InsertVertices_AsProvided(nExistingVerts); break;
            case VertexInsertionOrder.Auto:
                if (isFirstTime) InsertVertices_KDTreeBFS(nExistingVerts, box.min, box.max);
                else InsertVertices_Randomized(nExistingVerts); break;
            default: break;
        }
    }
    public void InsertEdges(Span<Edge> edges)
    {
        if (IsFinalized()) throw new ArgumentException("Triangulation was finalized with 'erase...' method. Inserting new edges is not possible");
        List<TriangulatePseudoPolygonTask> tppIterations = new(); List<Edge> remaining = new();
        for (int i = 0; i < edges.Length; i++)
        {
            Edge edge = new(edges[i].iV1 + nTargetVerts, edges[i].iV2 + nTargetVerts);
            InsertEdge(edge, edge, remaining, tppIterations);
        }
        EraseDummies();
    }
    public void InsertEdges(IEnumerable<Edge> edges)
    {
        if (IsFinalized()) throw new ArgumentException("Triangulation was finalized with 'erase...' method. Inserting new edges is not possible");
        List<TriangulatePseudoPolygonTask> tppIterations = new(); List<Edge> remaining = new();
        foreach (var i in edges)
        {
            Edge edge = new(i.iV1 + nTargetVerts, i.iV2 + nTargetVerts);
            InsertEdge(edge, edge, remaining, tppIterations);
        }
        EraseDummies();
    }
    public void ConformToEdges(Span<Edge> edges)
    {
        List<ConformToEdgeTask> remaining = new();
        if (IsFinalized()) throw new ArgumentException("Triangulation was finalized with 'erase...' method. Conforming to new edges is not possible");
        TryInitNearestPointLocator();
        for (int i = 0; i < edges.Length; i++)
        {
            Edge edge = new(edges[i].iV1 + nTargetVerts, edges[i].iV2 + nTargetVerts);
            ConformToEdge(edge, new() { edge }, 0, remaining);
        }
    }
    /// <summary>Ensure that triangulation conforms to constraints (fixed edges)</summary>
    /// <remarks>For each fixed edge that is not present in the triangulation its
    /// midpoint is recursively added until the original edge is represented by a
    /// sequence of its pieces. <b> New vertices are inserted.</b>
    /// If some edge appears more than once the input this
    /// means that multiple boundaries overlap at the edge and impacts how hole
    /// detection algorithm of Triangulation::eraseOuterTrianglesAndHoles works.
    /// <b>Make sure there are no erroneous duplicates.</b></remarks>
    /// <param name="edges">edges to conform to</param>
    public void ConformToEdges(IEnumerable<Edge> edges)
    {
        List<ConformToEdgeTask> remaining = new();
        if (IsFinalized()) throw new ArgumentException("Triangulation was finalized with 'erase...' method. Conforming to new edges is not possible");
        foreach (var i in edges)
        {
            Edge edge = new(i.iV1 + nTargetVerts, i.iV2 + nTargetVerts);
            ConformToEdge(edge, new() { edge }, 0, remaining);
        }
    }
    public void RefineTriangles(int maxVerticesToInsert, RefinementCriterion refinementCriterion = RefinementCriterion.SmallestAngle, double refinementThreshold = 20 / 180.0 * Math.PI)
    {
        if (IsFinalized()) throw new ArgumentException("Triangulation was finalized with 'erase...' method. Refinement is not possible");
        TryInitNearestPointLocator();
        int remainingVertexBudget = maxVerticesToInsert,
            steinerVerticesOffset = vertices.Count;
        Queue<Edge> encroachedEdges = FindEncroachedFixedEdges();
        for (; encroachedEdges.Count > 0 && remainingVertexBudget > 0; --remainingVertexBudget)
        {
            Edge edge = encroachedEdges.Dequeue();
            int iSplitVert = SplitEncroachedEdge(edge, steinerVerticesOffset);
            Edge half1 = new(edge.iV1, iSplitVert);
            if (IsEdgeEncroached(half1)) encroachedEdges.Enqueue(half1);
            Edge half2 = new(iSplitVert, edge.iV2);
            if (IsEdgeEncroached(half2)) encroachedEdges.Enqueue(half2);
        }
        if (remainingVertexBudget == 0) return;
        Queue<int> badTriangles = new();
        for (int iT = 0; iT < triangles.Count; iT++)
        {
            Triangle t = triangles[iT];
            if (!Triangle.TouchesSuperTriangle(t) && IsRefinementNeeded(t, refinementCriterion, refinementThreshold)) badTriangles.Enqueue(iT);
        }
        while (badTriangles.Count > 0 && remainingVertexBudget > 0)
        {
            int iT = badTriangles.Dequeue();
            Triangle badT = triangles[iT];
            if (!IsRefinementNeeded(badT, refinementCriterion, refinementThreshold)) continue;
            Vector2 triCircumcenter = Triangle.Circumcenter(vertices[badT.vertices[0]], vertices[badT.vertices[1]], vertices[badT.vertices[2]]);
            if (Triangle.LocatePointTriangle(triCircumcenter, vertices[0], vertices[1], vertices[2]) == PtTriLocation.Outside) continue;
            List<int> badTris = ResolveEncroachedEdges(EdgesEncroachedBy(triCircumcenter), ref remainingVertexBudget, steinerVerticesOffset, triCircumcenter, refinementCriterion, refinementThreshold);
            if (remainingVertexBudget == 0) break;
            if (badTris.Count > 0)
            {
                for (int it = 0; it < badTris.Count; it++) badTriangles.Enqueue(badTris[it]);
                badTriangles.Enqueue(iT); continue;
            }
            --remainingVertexBudget;
            int iVert = vertices.Count;
            AddNewVertex(triCircumcenter, noNeighbor);
            InsertVertex(iVert);
            int start = vertTris[iVert], currTri = start;
            do
            {
                Triangle t = triangles[currTri];
                if (IsRefinementNeeded(t, refinementCriterion, refinementThreshold)) badTriangles.Enqueue(currTri);
                currTri = t.Next(iVert).Item1;
            } while (currTri != start);
        }
    }
    /// <summary>Erase triangles adjacent to super triangle</summary>
    /// <remarks>does nothing if custom geometry is used</remarks>
    public void EraseSuperTriangle()
    {
        if (superGeomType != SuperGeometryType.SuperTriangle) return;
        HashSet<int> toErase = new(triangles.Count);
        for (int i = 0; i < triangles.Count; i++)
            if (Triangle.TouchesSuperTriangle(triangles[i])) toErase.Add(i);
        FinalizeTriangulation(toErase);
    }
    /// <summary>Erase triangles outside of constrained boundary using growing</summary>
    public void EraseOuterTriangles()
    {
        Debug.Assert(vertTris.Count > 0 && vertTris[0] != noNeighbor);
        Stack<int> seed = new(); seed.Push(vertTris[0]);
        HashSet<int> toErase = GrowToBoundary(seed);
        FinalizeTriangulation(toErase);
    }
    /// <summary>Erase triangles outside of constrained boundary and auto-detected holes</summary>
    /// <remarks>detecting holes relies on layer peeling based on layer depth<br/>supports overlapping or touching boundaries</remarks>
    public void EraseOuterTrianglesAndHoles()
    {
        List<ushort> triDepth = CalculateTriangleDepths(); HashSet<int> toErase = new(triangles.Count);
        for (int i = 0; i < triangles.Count; i++) if (triDepth[i] % 2 == 0) toErase.Add(i);
        FinalizeTriangulation(toErase);
    }
    static Edge RemapNoSuperTriangle(Edge e) => new(e.iV1 - 3, e.iV2 - 3);
    /// <summary>Call this method after directly setting custom super-geometry via vertices and triangles members</summary>
    public void InitializedWithCustomSuperGeometry()
    {
        nearPtLocator = new(vertices); nTargetVerts = vertices.Count; superGeomType = SuperGeometryType.Custom;
    }
    /// <summary>Check if the triangulation was finalized with `erase...` method and super-triangle was removed.</summary>
    /// <returns>true if triangulation is finalized, false otherwise</returns>
    public bool IsFinalized() => vertTris.Count == 0 && vertices.Count > 0;
    /// <summary>Calculate depth of each triangle in constraint triangulation. Supports
    /// <br/>overlapping boundaries.
    /// <br/>
    /// <br/>Perform depth peeling from super triangle to outermost boundary,
    /// <br/>then to next boundary and so on until all triangles are traversed.@n
    /// <br/>For example depth is:
    /// <br/> - 0 for triangles outside outermost boundary
    /// <br/> - 1 for triangles inside boundary but outside hole
    /// <br/> - 2 for triangles in hole
    /// <br/> - 3 for triangles in island and so on...
    /// <br/>@return vector where element at index i stores depth of i-th triangle
    /// </summary>
    public List<ushort> CalculateTriangleDepths()
    {
        List<ushort> triDepths = new(triangles.Count);
        Stack<int> seeds = new(); seeds.Push(vertTris[0]);
        ushort layerDepth = 0, deepestSeedDepth = 0;
        Dictionary<ushort, Stack<int>> seedsByDepth = new();
        do
        {
            Dictionary<int, ushort> newSeeds = PeelLayer(seeds, layerDepth, triDepths);
            seedsByDepth.Remove(layerDepth);
            foreach (var it in newSeeds)
            {
                deepestSeedDepth = Math.Max(deepestSeedDepth, it.Value);
                seedsByDepth[it.Value].Push(it.Key);
            }
            seeds = seedsByDepth[(ushort)(layerDepth + 1)]; layerDepth++;
        } while (seeds.Count > 0 || deepestSeedDepth > layerDepth);
        return triDepths;
    }
    /* Flip edge between T and Topo:
    *
    *                v4         | - old edge
    *               /|\         ~ - new edge
    *              / | \
    *          n3 /  T' \ n4
    *            /   |   \
    *           /    |    \
    *     T -> v1~~~~~~~~~v3 <- Topo
    *           \    |    /
    *            \   |   /
    *          n1 \Topo'/ n2
    *              \ | /
    *               \|/
    *                v2
    */
    /// <summary>Flip an edge between two triangle.</summary>
    /// <remarks>Advanced method for manually modifying the triangulation from outside. Please call it when you know what you are doing.</remarks>
    /// <param name="iT">first triangle</param>
    /// <param name="iTopo">second triangle</param>
    public void FlipEdge(int iT, int iTopo)
    {
        Triangle t = triangles[iT], tOpo = triangles[iTopo];
        int i = Triangle.OpposedVertexIndex(t.neighbors, iTopo);
        int v1 = t.vertices[i], v2 = t.vertices[Triangle.CCW(i)], n1 = t.neighbors[i], n3 = t.neighbors[Triangle.CW(i)];
        i = Triangle.OpposedVertexIndex(tOpo.neighbors, iT);
        int v3 = tOpo.vertices[i], v4 = tOpo.vertices[Triangle.CCW(i)], n4 = tOpo.neighbors[i], n2 = tOpo.neighbors[Triangle.CW(i)];
        triangles[iT] = new() { vertices = new[] { v4, v1, v3 }, neighbors = new[] { n3, iTopo, n4 } };
        triangles[iTopo] = new() { vertices = new[] { v2, v3, v1 }, neighbors = new[] { n2, iT, n1 } };
        ChangeNeighbor(n1, iT, iTopo); ChangeNeighbor(n4, iTopo, iT);
        if (!IsFinalized()) { SetAdjancentTriangle(v4, iT); SetAdjancentTriangle(v2, iTopo); }
    }
    /* Flip edge between T and Topo:
     *
     *                v4         | - old edge
     *               /|\         ~ - new edge
     *              / | \
     *          n3 /  T' \ n4
     *            /   |   \
     *           /    |    \
     *     T -> v1 ~~~~~~~~ v3 <- Topo
     *           \    |    /
     *            \   |   /
     *          n1 \Topo'/ n2
     *              \ | /
     *               \|/
     *                v2
     */
    public void FlipEdge(int iT, int iTopo, int v1, int v2, int v3, int v4, int n1, int n2, int n3, int n4)
    {
        triangles[iT] = new() { vertices = new[] { v4, v1, v3 }, neighbors = new[] { n3, iTopo, n4 } };
        triangles[iTopo] = new() { vertices = new[] { v2, v3, v1 }, neighbors = new[] { n2, iT, n1 } };
        ChangeNeighbor(n1, iT, iTopo); ChangeNeighbor(n4, iTopo, iT);
        if (!IsFinalized()) { SetAdjancentTriangle(v4, iT); SetAdjancentTriangle(v2, iTopo); }
    }
    public void RemoveTriangles(HashSet<int> removedTriangles)
    {
        if (removedTriangles.Count == 0) return; Dictionary<int, int> triIndMap = new();
        for (int iT = 0, iTnew = 0; iT < triangles.Count; iT++)
        {
            if (removedTriangles.Contains(iT)) continue;
            triIndMap[iT] = iTnew;
            triangles[iTnew] = triangles[iT];
            iTnew++;
        }
        triangles.RemoveRange(triangles.Count - removedTriangles.Count, removedTriangles.Count);
        for (int i = 0; i < triangles.Count; i++) for (int n = 0; n < 3; n++)
                if (removedTriangles.Contains(triangles[i].neighbors[n])) triangles[i].neighbors[n] = noNeighbor;
                else if (triangles[i].neighbors[n] != noNeighbor) triangles[i].neighbors[n] = triIndMap[triangles[i].neighbors[n]];
    }
    /// <summary>Access internal vertex adjacent triangles</summary>
    public List<int> VertTrisInternal() => vertTris;
    void AddSuperTriangle(Box box)
    {
        nTargetVerts = 3; superGeomType = SuperGeometryType.SuperTriangle;
        Vector2 center = (box.min + box.max) / 2;
        double r = Vector2.Distance(box.max, box.min) / 2;
        r = r > 0 ? r * 1.1 : 1e-6;
        while (center.Y <= center.Y - r) r *= 2;
        double R = r * 2, shiftX = R * Math.Sqrt(3) / 2;
        Vector2 posV1 = center - new Vector2(shiftX, r), posV2 = center + new Vector2(shiftX, -r), posV3 = center + new Vector2(0, R);
        AddNewVertex(posV1, 0); AddNewVertex(posV2, 0); AddNewVertex(posV3, 0);
        AddTriangle(new() { vertices = new[] { 0, 1, 2 }, neighbors = new[] { noNeighbor, noNeighbor, noNeighbor } });
        if (vertexInsertionOrder != VertexInsertionOrder.Auto) nearPtLocator = new(vertices);
    }
    void AddNewVertex(Vector2 pos, int iT) { vertices.Add(pos); vertTris.Add(iT); }
    void InsertVertex(int iVert)
    {
        Vector2 v = vertices[iVert]; int walkStart = nearPtLocator.Nearest(v, vertices).Item2;
        InsertVertex(iVert, walkStart); TryAddVertexToLocator(iVert);
    }
    void InsertVertex(int iVert, int walkStart)
    {
        int[] trisAt = WalkingSearchTriangleAt(iVert, walkStart);
        Stack<int> triStack = trisAt[1] == noNeighbor ? InsertVertexInsideTriangle(iVert, trisAt[0]) : InsertVertexOnEdge(iVert, trisAt[0], trisAt[1]);
        EnsureDelaunayByEdgeFlips(iVert, triStack);
    }
    void EnsureDelaunayByEdgeFlips(int iV1, Stack<int> triStack)
    {
        while (triStack.Count > 0)
        {
            int iT = triStack.Pop();
            EdgeFlipInfo(iT, iV1, out int iTopo, out int iV2, out int iV3, out int iV4, out int n1, out int n2, out int n3, out int n4);
            if (iTopo != noNeighbor && IsFlipNeeded(iV1, iV2, iV3, iV4))
            {
                FlipEdge(iT, iTopo, iV1, iV2, iV3, iV4, n1, n2, n3, n4); triStack.Push(iT); triStack.Push(iTopo);
            }
        }
    }
    List<Edge> InsertVertex_FlipFixedEdges(int iV1)
    {
        List<Edge> flippedFixedEdges = new(); Vector2 v1 = vertices[iV1]; int startVertex = nearPtLocator.Nearest(v1, vertices).Item2;
        int[] trisAt = WalkingSearchTriangleAt(iV1, startVertex);
        Stack<int> triStack = trisAt[1] == noNeighbor ? InsertVertexInsideTriangle(iV1, trisAt[0]) : InsertVertexOnEdge(iV1, trisAt[0], trisAt[1]);
        while (triStack.Count > 0)
        {
            int iT = triStack.Pop(); EdgeFlipInfo(iT, iV1, out int iTopo, out int iV2, out int iV3, out int iV4, out int n1, out int n2, out int n3, out int n4);
            if (iTopo != noNeighbor && IsFlipNeeded(iV1, iV2, iV3, iV4))
            {
                Edge flippedEdge = new(iV2, iV4);
                if (fixedEdges.Count > 0 && fixedEdges.Contains(flippedEdge)) flippedFixedEdges.Add(flippedEdge);
                FlipEdge(iT, iTopo, iV1, iV2, iV3, iV4, n1, n2, n3, n4); triStack.Push(iT); triStack.Push(iTopo);
            }
        }
        TryAddVertexToLocator(iV1); return flippedFixedEdges;
    }
    record TriangulatePseudoPolygonTask(int Size1, int Size2, int TriInd1, int TriInd2, int Index);
    /// <summary>Insert an edge into constraint Delaunay triangulation</summary>
    /// <param name="edge">edge to insert</param>
    /// <param name="originalEge">original edge inserted edge is part of</param>
    /// <param name="remaining">parts of the edge that still need to be inserted</param>
    /// <param name="tppIterations">stack to be used for storing iterations of triangulating pseudo-polygon</param>
    /// <remarks>in-out state (@param remaining @param tppIterations) is shared between different runs for performance gains (reducing memory allocations)</remarks>
    void InsertEdge(Edge edge, Edge originalEdge, List<Edge> remaining, List<TriangulatePseudoPolygonTask> tppIterations)
    {
        remaining.Clear(); remaining.Add(edge);
        while (remaining.Count > 0)
        {
            edge = remaining[^1]; remaining.RemoveAt(remaining.Count - 1);
            InsertEdgeIteration(edge, originalEdge, remaining, tppIterations);
        }
    }
    static double Lerp(double a, double b, double t) => (1 - t) * a + t * b;
    static Vector2 IntersectionPosition(Vector2 a, Vector2 b, Vector2 c, Vector2 d)
    {
        double a_cd = Triangle.Orient2D(c.X, c.Y, d.X, d.Y, a.X, a.Y), b_cd = Triangle.Orient2D(c.X, c.Y, d.X, d.Y, b.X, b.Y), t_ab = a_cd / (a_cd - b_cd),
            c_ab = Triangle.Orient2D(a.X, a.Y, b.X, b.Y, c.X, c.Y), d_ab = Triangle.Orient2D(a.X, a.Y, b.X, b.Y, d.X, d.Y), t_cd = c_ab / (c_ab - d_ab);
        return new(Math.Abs(a.X - b.X) < Math.Abs(c.X - d.X) ? Lerp(a.X, b.X, t_ab) : Lerp(c.X, d.X, t_cd),
            Math.Abs(a.Y - b.Y) < Math.Abs(c.Y - d.Y) ? Lerp(a.Y, b.Y, t_ab) : Lerp(c.Y, d.Y, t_cd));
    }
    void InsertEdgeIteration(Edge edge, Edge originalEdge, List<Edge> remaining, List<TriangulatePseudoPolygonTask> tppIterations)
    {
        int iA = edge.iV1, iB = edge.iV2; if (iA == iB) return;
        if (HasEdge(iA, iB)) { FixEdge(edge, originalEdge); return; }
        Vector2 a = vertices[iA], b = vertices[iB]; double distanceTolerance = minDistToConstraintEdge == 0 ? 0 : minDistToConstraintEdge * Vector2.Distance(a, b);
        (int iT, int iVL, int iVR) = IntersectedTriangle(iA, a, b, distanceTolerance);
        if (iT == noNeighbor)
        { Edge edgePart = new(iA, iVL); FixEdge(edgePart, originalEdge); remaining.Add(new(iVL, iB)); return; }
        Triangle t = triangles[iT];
        List<int> intersected = new() { iT }, polyL = new() { iA, iVL }, polyR = new() { iA, iVR };
        Dictionary<Edge, int> outerTris = new() { { new(iA, iVL), Triangle.EdgeNeighbor(t, iA, iVL) }, { new(iA, iVR), Triangle.EdgeNeighbor(t, iA, iVR) } };
        int iV = iA;
        while (!t.ContainsVertex(iB))
        {
            int iTopo = Triangle.OpposedTriangle(t, iV); Triangle tOpo = triangles[iTopo];
            int iVopo = Triangle.OpposedVertex(tOpo, iT);
            switch (intersectingEdgesStrategy)
            {
                case IntersectingConstraintEdges.NotAllowed:
                    if (fixedEdges.Contains(new(iVL, iVR)))
                    {
                        Edge e1 = originalEdge, e2 = new(iVL, iVR);
                        e2 = pieceToOriginals.ContainsKey(e2) ? pieceToOriginals[e2][0] : e2;
                        e1 = new(e1.iV1 - nTargetVerts, e1.iV2 - nTargetVerts);
                        e2 = new(e2.iV1 - nTargetVerts, e2.iV2 - nTargetVerts);
                        throw new Exception("Intersecting constraints error");
                    }
                    break;
                case IntersectingConstraintEdges.TryResolve:
                    if (!fixedEdges.Contains(new(iVL, iVR))) break;
                    Vector2 newV = IntersectionPosition(vertices[iA], vertices[iB], vertices[iVL], vertices[iVR]);
                    int iNewVert = SplitFixedEdgeAt(new(iVL, iVR), newV, iT, iTopo);
                    remaining.Add(new(iA, iNewVert)); remaining.Add(new(iNewVert, iB)); return;
                case IntersectingConstraintEdges.DontCheck:
                    Debug.Assert(!fixedEdges.Contains(new(iVL, iVR)));
                    break;
                default:
                    break;
            }
            PtLineLocation loc = Triangle.LocatePointLine(vertices[iVopo], a, b, distanceTolerance);
            if (loc == PtLineLocation.Left)
            {
                Edge e = new(polyL[^1], iVopo);
                int outer = Triangle.EdgeNeighbor(tOpo, e.iV1, e.iV2);
                if (!outerTris.TryAdd(e, outer)) outerTris[e] = noNeighbor;
                polyL.Add(iVopo); iV = iVL; iVL = iVopo;
            }
            else if (loc == PtLineLocation.Right)
            {
                Edge e = new(polyR[^1], iVopo);
                int outer = Triangle.EdgeNeighbor(tOpo, polyR[^1], iVopo);
                if (!outerTris.TryAdd(e, outer)) outerTris[e] = noNeighbor;
                polyR.Add(iVopo); iV = iVR; iVR = iVopo;
            }
            else iB = iVopo;
            intersected.Add(iTopo);
            iT = iTopo; t = triangles[iT];
        }
        outerTris[new(polyL[^1], iB)] = Triangle.EdgeNeighbor(t, polyL[^1], iB);
        outerTris[new(polyR[^1], iB)] = Triangle.EdgeNeighbor(t, polyR[^1], iB);
        polyL.Add(iB); polyR.Add(iB);
        Debug.Assert(intersected.Count > 0);
        if (vertTris[iA] == intersected[0]) PivotVertexTriangleCW(iA);
        if (vertTris[iB] == intersected[^1]) PivotVertexTriangleCW(iB);
        foreach (var i in intersected) MakeDummy(i);
        polyR.Reverse();
        int iTL = AddTriangle(), iTR = AddTriangle();
        TriangulatePseudoPolygon(polyL, outerTris, iTL, iTR, tppIterations);
        TriangulatePseudoPolygon(polyR, outerTris, iTR, iTL, tppIterations);
        if (iB != edge.iV2)
        {
            Edge edgePart = new(iA, iB); FixEdge(edgePart, originalEdge);
            remaining.Add(new(iB, edge.iV2)); return;
        }
        else FixEdge(edge, originalEdge);
    }
    record ConformToEdgeTask(Edge edge, List<Edge> EdgeVec, ushort BoundaryOverlapCount);
    /// <summary>Conform Delaunay triangulation to a fixed edge by recursively inserting mid point of the edge and then conforming to its halves</summary>
    /// <param name="edge">fixed edge to conform to</param>
    /// <param name="originals">original edges that new edge is piece of</param>
    /// <param name="overlaps">count of overlapping boundaries at the edge. Only used when re-introducing edge with overlaps > 0</param>
    /// <param name="remaining">remaining remaining edge parts to be conformed to</param>
    /// <remarks>in-out state (@param remaining @param reintroduce) is shared between different runs for performance gains (reducing memory allocations)</remarks>
    void ConformToEdge(Edge edge, List<Edge> originals, ushort overlaps, List<ConformToEdgeTask> remaining)
    {
        remaining.Clear(); remaining.Add(new(edge, originals, overlaps));
        while (remaining.Count > 0)
        {
            (edge, originals, overlaps) = remaining[^1];
            remaining.RemoveAt(remaining.Count - 1); ConformToEdgeIteration(edge, originals, overlaps, remaining);
        }
    }
    /// <summary>Iteration of conform to fixed edge.</summary>
    /// <param name="edge">fixed edge to conform to</param>
    /// <param name="originals">original edges that new edge is piece of</param>
    /// <param name="overlaps">count of overlapping boundaries at the edge. Only used when re-introducing edge with overlaps > 0</param>
    /// <param name="remaining">remaining remaining edge parts to be conformed to</param>
    /// <remarks>in-out state (@param remaining @param reintroduce) is shared between different runs for performance gains (reducing memory allocations)</remarks>
    void ConformToEdgeIteration(Edge edge, List<Edge> originals, ushort overlaps, List<ConformToEdgeTask> remaining)
    {
        int iA = edge.iV1, iB = edge.iV2; if (iA == iB) return;
        if (HasEdge(iA, iB))
        {
            FixEdge(edge);
            if (overlaps > 0) overlapCount[edge] = overlaps;
            if (originals.Count > 0 && edge != originals[0]) DictionaryInsertUnique(pieceToOriginals, edge, originals);
            return;
        }
        Vector2 a = vertices[iA], b = vertices[iB]; double distanceTolerance = minDistToConstraintEdge == 0 ? 0 : minDistToConstraintEdge * Vector2.Distance(a, b);
        (int iT, int iVleft, int iVright) = IntersectedTriangle(iA, a, b, distanceTolerance);
        if (iT == noNeighbor)
        {
            Edge edgePart = new(iA, iVleft); FixEdge(edgePart);
            if (overlaps > 0) overlapCount[edgePart] = overlaps;
            DictionaryInsertUnique(pieceToOriginals, edgePart, originals);
            remaining.Add(new(new(iVleft, iB), originals, overlaps));
            return;
        }
        int iV = iA; Triangle t = triangles[iT];
        while (t.vertices[0] != iB && t.vertices[1] != iB && t.vertices[2] != iB)
        {
            int iTopo = Triangle.OpposedTriangle(t, iV); Triangle tOpo = triangles[iTopo];
            int iVopo = Triangle.OpposedVertex(tOpo, iT); Vector2 vOpo = vertices[iVopo];
            switch (intersectingEdgesStrategy)
            {
                case IntersectingConstraintEdges.NotAllowed:
                    if (fixedEdges.Contains(new(iVleft, iVright)))
                    {
                        Edge e1 = pieceToOriginals.ContainsKey(edge) ? pieceToOriginals[edge][0] : edge,
                            e2 = new(iVleft, iVright);
                        e2 = pieceToOriginals.ContainsKey(e2) ? pieceToOriginals[e2][0] : e2;
                        e1 = new(e1.iV1 - nTargetVerts, e1.iV2 - nTargetVerts);
                        e2 = new(e2.iV1 - nTargetVerts, e2.iV2 - nTargetVerts);
                        throw new Exception("Intersecting constraints error");
                    }
                    break;
                case IntersectingConstraintEdges.TryResolve:
                    if (!fixedEdges.Contains(new(iVleft, iVright))) break;
                    Vector2 newV = IntersectionPosition(vertices[iA], vertices[iB], vertices[iVleft], vertices[iVright]);
                    int iNewVert = SplitFixedEdgeAt(new(iVleft, iVright), newV, iT, iTopo);
                    remaining.Add(new(new(iNewVert, iB), originals, overlaps));
                    remaining.Add(new(new(iA, iNewVert), originals, overlaps));
                    return;
                case IntersectingConstraintEdges.DontCheck:
                    Debug.Assert(!fixedEdges.Contains(new(iVleft, iVright)));
                    break;
                default:
                    break;
            }
            iT = iTopo; t = triangles[iT];
            PtLineLocation loc = Triangle.LocatePointLine(vOpo, a, b, distanceTolerance);
            if (loc == PtLineLocation.Left) { iV = iVleft; iVleft = iVopo; }
            else if (loc == PtLineLocation.Right) { iV = iVright; iVright = iVopo; }
            else iB = iVopo;
        }
        if (iB != edge.iV2) remaining.Add(new(new(iB, edge.iV2), originals, overlaps));
        int iMid = vertices.Count; Vector2 start = vertices[iA], end = vertices[iB];
        AddNewVertex((start + end) / 2, noNeighbor);
        List<Edge> flippedFixedEdges = InsertVertex_FlipFixedEdges(iMid);
        remaining.Add(new(new(iMid, iB), originals, overlaps));
        remaining.Add(new(new(iA, iMid), originals, overlaps));
        for (int i = 0; i < flippedFixedEdges.Count; i++)
        {
            fixedEdges.Remove(flippedFixedEdges[i]); ushort prevOverlaps = 0;
            if (overlapCount.TryGetValue(flippedFixedEdges[i], out var overlapsIt))
            {
                prevOverlaps = overlapsIt; overlapCount.Remove(flippedFixedEdges[i]);
            }
            List<Edge> prevOriginals = new() { flippedFixedEdges[i] };
            if (pieceToOriginals.TryGetValue(flippedFixedEdges[i], out var originalsIt))
                prevOriginals = originalsIt;
            remaining.Add(new(flippedFixedEdges[i], prevOriginals, prevOverlaps));
        }
    }
    (int, int, int) IntersectedTriangle(int iA, Vector2 a, Vector2 b, double orientationTolerance = 0)
    {
        int startTri = vertTris[iA], iT = startTri;
        do
        {
            Triangle t = triangles[iT];
            int i = Triangle.VertexIndex(t.vertices, iA), iP2 = t.vertices[Triangle.CCW(i)];
            double orientP2 = Triangle.Orient2D(a.X, a.Y, b.X, b.Y, vertices[iP2].X, vertices[iP2].Y);
            PtLineLocation locP2 = Triangle.ClassifyOrientation(orientP2);
            if (locP2 == PtLineLocation.Right)
            {
                int iP1 = t.vertices[Triangle.CW(i)];
                double orientP1 = Triangle.Orient2D(a.X, a.Y, b.X, b.Y, vertices[iP1].X, vertices[iP1].Y);
                PtLineLocation locP1 = Triangle.ClassifyOrientation(orientP1);
                if (locP1 == PtLineLocation.OnLine) return (noNeighbor, iP1, iP1);
                if (locP1 == PtLineLocation.Left)
                {
                    if (orientationTolerance > 0)
                    {
                        double closestOrient; int iClosestP;
                        if (Math.Abs(orientP1) <= Math.Abs(orientP2)) { closestOrient = orientP1; iClosestP = iP2; }
                        else { closestOrient = orientP2; iClosestP = iP2; }
                        if (Triangle.ClassifyOrientation(closestOrient, orientationTolerance) == PtLineLocation.OnLine) return (noNeighbor, iClosestP, iClosestP);
                    }
                    return (iT, iP1, iP2);
                }
            }
            iT = t.Next(iA).Item1;
        } while (iT != startTri);
        throw new MissingMemberException("Could not find vertex triangle intersected by edge. Note: can be caused by duplicate points.");
    }
    /* Insert point into triangle: split into 3 triangles:
    *  - create 2 new triangles
    *  - re-use old triangle for the 3rd
    *                      v3
    *                    / | \
    *                   /  |  \ <-- original triangle (t)
    *                  /   |   \
    *              n3 /    |    \ n2
    *                /newT2|newT1\
    *               /      v      \
    *              /    __/ \__    \
    *             /  __/       \__  \
    *            / _/      t'     \_ \
    *          v1 ___________________ v2
    *                     n1
    */
    Stack<int> InsertVertexInsideTriangle(int v, int iT)
    {
        int iNewT1 = AddTriangle(), iNewT2 = AddTriangle();
        Triangle t = triangles[iT];
        int[] vv = t.vertices, nn = t.neighbors;
        triangles[iNewT1] = new() { vertices = new[] { vv[1], vv[2], v }, neighbors = new[] { nn[1], iNewT2, iT } };
        triangles[iNewT2] = new() { vertices = new[] { vv[2], vv[0], v }, neighbors = new[] { nn[2], iT, iNewT1 } };
        triangles[iT] = new() { vertices = new[] { vv[0], vv[1], v }, neighbors = new[] { nn[0], iNewT1, iNewT2 } };
        SetAdjancentTriangle(v, iT); SetAdjancentTriangle(vv[2], iNewT1);
        ChangeNeighbor(nn[1], iT, iNewT1); ChangeNeighbor(nn[2], iT, iNewT2);
        Stack<int> newTriangles = new(); newTriangles.Push(iT); newTriangles.Push(iNewT1); newTriangles.Push(iNewT2);
        return newTriangles;
    }
    /* Inserting a point on the edge between two triangles
    *    T1 (top)        v1
    *                   /|\
    *              n1 /  |  \ n4
    *               /    |    \
    *             /  T1' | Tnew1\
    *           v2-------v-------v4
    *             \  T2' | Tnew2/
    *               \    |    /
    *              n2 \  |  / n3
    *                   \|/
    *   T2 (bottom)      v3
    */
    Stack<int> InsertVertexOnEdge(int v, int iT1, int iT2)
    {
        int iTnew1 = AddTriangle(), iTnew2 = AddTriangle();
        Triangle t1 = triangles[iT1], t2 = triangles[iT2];
        int i = Triangle.OpposedVertexIndex(t1.neighbors, iT2), v1 = t1.vertices[i], v2 = t1.vertices[Triangle.CCW(i)], n1 = t1.neighbors[i], n4 = t1.neighbors[Triangle.CW(i)];
        i = Triangle.OpposedVertexIndex(t2.neighbors, iT1); int v3 = t2.vertices[i], v4 = t2.vertices[Triangle.CCW(i)], n3 = t2.neighbors[i], n2 = t2.neighbors[Triangle.CW(i)];
        triangles[iT1] = new() { vertices = new[] { v, v1, v2 }, neighbors = new[] { iTnew1, n1, iT2 } };
        triangles[iT2] = new() { vertices = new[] { v, v2, v3 }, neighbors = new[] { iT1, n2, iTnew2 } };
        triangles[iTnew1] = new() { vertices = new[] { v, v4, v1 }, neighbors = new[] { iTnew2, n4, iT1 } };
        triangles[iTnew2] = new() { vertices = new[] { v, v3, v4 }, neighbors = new[] { iT2, n3, iTnew1 } };
        SetAdjancentTriangle(v, iT1); SetAdjancentTriangle(v4, iTnew1);
        ChangeNeighbor(n4, iT1, iTnew1); ChangeNeighbor(n3, iT2, iTnew2);
        Stack<int> newTriangles = new();
        newTriangles.Push(iT1); newTriangles.Push(iTnew2);
        newTriangles.Push(iT2); newTriangles.Push(iTnew1); return newTriangles;
    }
    int[] TrianglesAt(Vector2 pos)
    {
        int[] out_ = new int[] { noNeighbor, noNeighbor };
        for (int i = 0; i < triangles.Count; i++)
        {
            Triangle t = triangles[i]; Vector2 v1 = vertices[t.vertices[0]], v2 = vertices[t.vertices[1]], v3 = vertices[t.vertices[2]];
            PtTriLocation loc = Triangle.LocatePointTriangle(pos, v1, v2, v3);
            if (loc == PtTriLocation.Outside) continue;
            out_[0] = i; if (Triangle.IsOnEdge(loc)) out_[1] = t.neighbors[Triangle.EdgeNeighbor(loc)];
            return out_;
        }
        throw new MissingMemberException("No triangle was found at position");
    }
    int WalkTriangles(int startVertex, Vector2 pos)
    {
        int currTri = vertTris[startVertex]; bool found = false;
        SplitMix64RandGen prng = new();
        while (!found)
        {
            Triangle t = triangles[currTri]; found = true; int offset = (int)(prng.Next() % 3);
            for (int i_ = 0; i_ < 3; i_++)
            {
                int i = (i_ + offset) % 3;
                Vector2 vStart = vertices[t.vertices[i]], vEnd = vertices[t.vertices[Triangle.CCW(i)]];
                PtLineLocation edgeCheck = Triangle.LocatePointLine(pos, vStart, vEnd);
                int iN = t.neighbors[i];
                if (edgeCheck == PtLineLocation.Right && iN != noNeighbor)
                { found = false; currTri = iN; break; }
            }
        }
        return currTri;
    }
    int[] WalkingSearchTriangleAt(int iV, int startVertex)
    {
        int[] out_ = new int[] { noNeighbor, noNeighbor };
        Vector2 v = vertices[iV];
        int iT = WalkTriangles(startVertex, v);
        Triangle t = triangles[iT];
        Vector2 v1 = vertices[t.vertices[0]], v2 = vertices[t.vertices[1]], v3 = vertices[t.vertices[2]];
        PtTriLocation loc = Triangle.LocatePointTriangle(v, v1, v2, v3);
        if (loc == PtTriLocation.Outside) throw new MissingMemberException("No triangle was found at position");
        if (loc == PtTriLocation.OnVertex)
        {
            int iDupe = v1 == v ? t.vertices[0] : v2 == v ? t.vertices[1] : t.vertices[2];
            throw new DuplicateWaitObjectException($"Duplicate vertex error: {iV - nTargetVerts}, {iDupe - nTargetVerts}");
        }
        out_[0] = iT; if (Triangle.IsOnEdge(loc)) out_[1] = t.neighbors[Triangle.EdgeNeighbor(loc)];
        return out_;
    }
    /*
    *                       v4         original edge: (v1, v3)
    *                      /|\   flip-candidate edge: (v,  v2)
    *                    /  |  \
    *              n3  /    |    \  n4
    *                /      |      \
    * new vertex--> v1    T | Topo  v3
    *                \      |      /
    *              n1  \    |    /  n2
    *                    \  |  /
    *                      \|/
    *                       v2
    */
    void EdgeFlipInfo(int iT, int iV1, out int iTopo, out int iV2, out int iV3, out int iV4, out int n1, out int n2, out int n3, out int n4)
    {
        Triangle t = triangles[iT];
        if (t.vertices[0] == iV1)
        {
            iV2 = t.vertices[1];
            iV4 = t.vertices[2];
            n1 = t.neighbors[0];
            n3 = t.neighbors[2];
            iTopo = t.neighbors[1];
        }
        else if (t.vertices[1] == iV1)
        {
            iV2 = t.vertices[2];
            iV4 = t.vertices[0];
            n1 = t.neighbors[1];
            n3 = t.neighbors[0];
            iTopo = t.neighbors[2];
        }
        else
        {
            iV2 = t.vertices[0];
            iV4 = t.vertices[1];
            n1 = t.neighbors[2];
            n3 = t.neighbors[1];
            iTopo = t.neighbors[0];
        }
        iV3 = 0; n2 = 0; n4 = 0;
        if (iTopo == noNeighbor) return;
        Triangle tOpo = triangles[iTopo];
        if (tOpo.neighbors[0] == iT)
        {
            iV3 = tOpo.vertices[2];
            n2 = tOpo.neighbors[1];
            n4 = tOpo.neighbors[2];
        }
        else if (tOpo.neighbors[1] == iT)
        {
            iV3 = tOpo.vertices[0];
            n2 = tOpo.neighbors[2];
            n4 = tOpo.neighbors[0];
        }
        else
        {
            iV3 = tOpo.vertices[1];
            n2 = tOpo.neighbors[0];
            n4 = tOpo.neighbors[1];
        }
    }
    /*!
    * Handles super-triangle vertices.
    * Super-tri points are not infinitely far and influence the input points
    * Three cases are possible:
    *  1.  If one of the opposed vertices is super-tri: no flip needed
    *  2.  One of the shared vertices is super-tri:
    *      check if on point is same side of line formed by non-super-tri
    * vertices as the non-super-tri shared vertex
    *  3.  None of the vertices are super-tri: normal circumcircle test
    */
    /*
    *                       v4         original edge: (v2, v4)
    *                      /|\   flip-candidate edge: (v1, v3)
    *                    /  |  \
    *                  /    |    \
    *                /      |      \
    * new vertex--> v1      |       v3
    *                \      |      /
    *                  \    |    /
    *                    \  |  /
    *                      \|/
    *                       v2
    */
    bool IsFlipNeeded(int iV1, int iV2, int iV3, int iV4)
    {
        if (fixedEdges.Contains(new(iV2, iV4))) return false;
        Vector2 v1 = vertices[iV1], v2 = vertices[iV2], v3 = vertices[iV3], v4 = vertices[iV4];
        if (superGeomType == SuperGeometryType.SuperTriangle)
        {
            // If flip-candidate edge touches super-triangle in-circumference
            // test has to be replaced with orient2d test against the line
            // formed by two non-artificial vertices (that don't belong to
            // super-triangle)
            if (iV1 < 3) // flip-candidate edge touches super-triangle
            {
                // does original edge also touch super-triangle?
                if (iV2 < 3)
                    return Triangle.LocatePointLine(v2, v3, v4) ==
                           Triangle.LocatePointLine(v1, v3, v4);
                if (iV4 < 3)
                    return Triangle.LocatePointLine(v4, v2, v3) ==
                           Triangle.LocatePointLine(v1, v2, v3);
                return false; // original edge does not touch super-triangle
            }
            if (iV3 < 3) // flip-candidate edge touches super-triangle
            {
                // does original edge also touch super-triangle?
                if (iV2 < 3)
                    return Triangle.LocatePointLine(v2, v1, v4) == Triangle.LocatePointLine(v3, v1, v4);
                if (iV4 < 3)
                    return Triangle.LocatePointLine(v4, v2, v1) == Triangle.LocatePointLine(v3, v2, v1);
                return false; // original edge does not touch super-triangle
            }
            // flip-candidate edge does not touch super-triangle
            if (iV2 < 3)
                return Triangle.LocatePointLine(v2, v3, v4) == Triangle.LocatePointLine(v1, v3, v4);
            if (iV4 < 3)
                return Triangle.LocatePointLine(v4, v2, v3) == Triangle.LocatePointLine(v1, v2, v3);
        }
        return Triangle.IsInCircumcircle(v1, v2, v3, v4);
    }
    bool IsRefinementNeeded(Triangle tri, RefinementCriterion refinementCriterion, double refinementThreshold)
    {
        Vector2 a = vertices[tri.vertices[0]], b = vertices[tri.vertices[1]], c = vertices[tri.vertices[2]];
        switch (refinementCriterion)
        {
            case RefinementCriterion.SmallestAngle:
                return Triangle.SmallestAngle(a, b, c) <= refinementThreshold;
            case RefinementCriterion.LargestArea:
                return Triangle.Area(a, b, c) >= refinementThreshold;
            default: return false;
        }
    }
    bool IsEdgeEncroached(Edge edge)
    {
        (int iT, int iTopo) = EdgeTriangles(edge.iV1, edge.iV2);
        Debug.Assert(iT != invalidIndex && iTopo != invalidIndex);
        int v1 = Triangle.OpposedVertex(triangles[iT], iTopo),
            v2 = Triangle.OpposedVertex(triangles[iTopo], iT);
        Vector2 edgeStart = vertices[edge.iV1], edgeEnd = vertices[edge.iV2];
        return Triangle.IsEncroachingOnEdge(vertices[v1], edgeStart, edgeEnd) || Triangle.IsEncroachingOnEdge(vertices[v2], edgeStart, edgeEnd);
    }
    bool IsEdgeEncroachedBy(Edge edge, Vector2 v) => Triangle.IsEncroachingOnEdge(v, vertices[edge.iV1], vertices[edge.iV2]);
    Queue<Edge> FindEncroachedFixedEdges()
    {
        Queue<Edge> encroachedEdges = new();
        foreach (var edge in fixedEdges)
        {
            if (IsEdgeEncroached(edge)) encroachedEdges.Enqueue(edge);
        }
        return encroachedEdges;
    }
    Queue<Edge> EdgesEncroachedBy(Vector2 v)
    {
        Queue<Edge> encroachedEdges = new();
        foreach (var it in fixedEdges)
        {
            if (IsEdgeEncroachedBy(it, v)) encroachedEdges.Enqueue(it);
        }
        return encroachedEdges;
    }
    List<int> ResolveEncroachedEdges(Queue<Edge> encroachedEdges, ref int remainingVertexBudget, int steinerVerticesOffset,
        Vector2? circumcenterOrNull, RefinementCriterion refinementCriterion, double badTriangleThreshold)
    {
        List<int> badTriangles = new();
        while (encroachedEdges.Count > 0 && remainingVertexBudget > 0)
        {
            Edge edge = encroachedEdges.Dequeue();
            if (!fixedEdges.Contains(edge)) continue;
            int iSplitVert = SplitEncroachedEdge(edge, steinerVerticesOffset);
            --remainingVertexBudget;
            int start = vertTris[iSplitVert];
            int iT = start;
            do
            {
                Triangle t = triangles[iT];
                if (circumcenterOrNull != null && IsRefinementNeeded(t, refinementCriterion, badTriangleThreshold)) badTriangles.Add(iT);
                for (int i = 0; i < 3; i++)
                {
                    Edge triEdge = new(t.vertices[i], t.vertices[Triangle.CW(i)]);
                    if (!fixedEdges.Contains(triEdge)) continue;
                    if (IsEdgeEncroached(triEdge) || (circumcenterOrNull is Vector2 v && IsEdgeEncroachedBy(triEdge, v))) encroachedEdges.Enqueue(triEdge);
                }
                iT = t.Next(iSplitVert).Item1;
            } while (iT != start);
        }
        return badTriangles;
    }
    int SplitEncroachedEdge(Edge edge, int steinerVerticesOffset)
    {
        Vector2 start = vertices[edge.iV1], end = vertices[edge.iV2]; double split = 0.5;
        if (edge.iV1 >= steinerVerticesOffset || edge.iV2 >= steinerVerticesOffset)
        {
            double len = Vector2.Distance(start, end);
            double d = len / 2, nearestPowerOfTwo = 1;
            while (d > nearestPowerOfTwo) nearestPowerOfTwo *= 2;
            while (d < 0.75 * nearestPowerOfTwo) nearestPowerOfTwo *= 0.5;
            Debug.Assert(Math.Abs(nearestPowerOfTwo - Math.Pow(2, Math.Round(Math.Log2(d)))) < 1e6);
            split = nearestPowerOfTwo / len;
            if (edge.iV1 >= steinerVerticesOffset) split = 1 - split;
        }
        Vector2 mid = new(Lerp(start.X, end.X, split), Lerp(start.Y, end.Y, split));
        (int iT, int iTopo) = EdgeTriangles(edge.iV1, edge.iV2);
        Debug.Assert(iT != invalidIndex && iTopo != invalidIndex);
        int iMid = AddSplitEdgeVertex(mid, iT, iTopo);
        if (!fixedEdges.Contains(edge)) SplitFixedEdge(edge, iMid);
        return iMid;
    }
    void ChangeNeighbor(int iT, int oldNeighbor, int newNeighbor)
    {
        if (iT == noNeighbor) return;
        Debug.Assert(triangles[iT].neighbors[0] == oldNeighbor || triangles[iT].neighbors[1] == oldNeighbor || triangles[iT].neighbors[2] == oldNeighbor);
        if (triangles[iT].neighbors[0] == oldNeighbor) triangles[iT].neighbors[0] = newNeighbor;
        else if (triangles[iT].neighbors[1] == oldNeighbor) triangles[iT].neighbors[1] = newNeighbor;
        else triangles[iT].neighbors[2] = newNeighbor;
    }
    void ChangeNeighbor(int iT, int iVedge1, int iVedge2, int newNeighbor)
    {
        Debug.Assert(iT != noNeighbor); triangles[iT].neighbors[Triangle.EdgeNeighborIndex(triangles[iT].vertices, iVedge1, iVedge2)] = newNeighbor;
    }
    void TriangulatePseudoPolygon(List<int> poly, Dictionary<Edge, int> outerTris, int iT, int iN, List<TriangulatePseudoPolygonTask> iterations)
    {
        Debug.Assert(poly.Count > 2);
        iterations.Clear();
        iterations.Add(new(0, poly.Count - 1, iT, iN, 0));
        while (iterations.Count > 0) TriangulatePseudoPolygonIteration(poly, outerTris, iterations);
    }
    void TriangulatePseudoPolygonIteration(List<int> poly, Dictionary<Edge, int> outerTris, List<TriangulatePseudoPolygonTask> iterations)
    {
        Debug.Assert(iterations.Count > 0);
        (int iA, int iB, int iT, int iParent, int iInParent) = iterations[^1]; iterations.RemoveAt(iterations.Count - 1);
        Debug.Assert(iB - iA > 1 && iT != noNeighbor && iParent != noNeighbor);
        int iC = FindDelaunayPoint(poly, iA, iB), a = poly[iA], b = poly[iB], c = poly[iC];
        if (iB - iC > 1) { int iNext = AddTriangle(); iterations.Add(new(iC, iB, iNext, iT, 1)); }
        else
        {
            Edge outerEdge = new(b, c); int outerTri = outerTris[outerEdge];
            if (outerTri != noNeighbor)
            {
                Debug.Assert(outerTri != iT);
                triangles[iT].neighbors[1] = outerTri;
                ChangeNeighbor(outerTri, c, b, iT);
            }
            else outerTris[outerEdge] = iT;
        }
        if (iC - iA > 1) { int iNext = AddTriangle(); iterations.Add(new(iA, iC, iNext, iT, 2)); }
        else
        {
            Edge outerEdge = new(c, a); int outerTri = outerTris[outerEdge];
            if (outerTri != noNeighbor)
            {
                Debug.Assert(outerTri != iT);
                triangles[iT].neighbors[2] = outerTri;
                ChangeNeighbor(outerTri, c, a, iT);
            }
            else outerTris[outerEdge] = iT;
        }
        triangles[iParent].neighbors[iInParent] = iT;
        triangles[iT].neighbors[0] = iParent;
        triangles[iT].vertices = new[] { a, b, c };
        SetAdjancentTriangle(c, iT);
    }
    int FindDelaunayPoint(List<int> poly, int iA, int iB)
    {
        Debug.Assert(iB - iA > 1);
        Vector2 a = vertices[poly[iA]], b = vertices[poly[iB]];
        int out_ = iA + 1;
        Vector2 c = vertices[poly[out_]];
        for (int i = iA + 1; i < iB; i++)
        {
            Vector2 v = vertices[poly[i]];
            if (Triangle.IsInCircumcircle(v, a, b, c)) { out_ = i; c = v; }
        }
        Debug.Assert(out_ > iA && out_ < iB); return out_;
    }
    int AddTriangle(Triangle t)
    {
        if (dummyTris.Count == 0) { triangles.Add(t); return triangles.Count - 1; }
        int nxtDummy = dummyTris[^1];
        dummyTris.RemoveAt(dummyTris.Count - 1); triangles[nxtDummy] = t; return nxtDummy;
    }
    int AddTriangle()
    {
        if (dummyTris.Count == 0)
        {
            Triangle dummy = new() { vertices = new[] { noNeighbor, noNeighbor, noNeighbor }, neighbors = new[] { noNeighbor, noNeighbor, noNeighbor } };
            triangles.Add(dummy); return triangles.Count - 1;
        }
        int nxtDummy = dummyTris[^1]; dummyTris.RemoveAt(dummyTris.Count - 1); return nxtDummy;
    }
    /// <summary>Remove super-triangle (if used) and triangles with specified indices. Adjust internal triangulation state accordingly.</summary>
    /// <param name="removedTriangles">indices of triangles to remove</param>
    void FinalizeTriangulation(HashSet<int> removedTriangles)
    {
        EraseDummies(); vertTris.Clear();
        if (superGeomType == SuperGeometryType.SuperTriangle)
        {
            vertices.RemoveRange(0, 3);
            fixedEdges = fixedEdges.Select(x => RemapNoSuperTriangle(x)).ToHashSet();
            overlapCount = overlapCount.Select(x => (RemapNoSuperTriangle(x.Key), x.Value)).ToDictionary(x => x.Item1, x => x.Value);
            pieceToOriginals = pieceToOriginals.Select(x =>
            (RemapNoSuperTriangle(x.Key), x.Value.Select(x => RemapNoSuperTriangle(x)).ToList())).ToDictionary(x => x.Item1, x => x.Item2);
        }
        RemoveTriangles(removedTriangles);
        if (superGeomType == SuperGeometryType.SuperTriangle)
            for (int t = 0; t < triangles.Count; t++) for (int i = 0; i < 3; i++) triangles[t].vertices[i] -= 3;
    }
    HashSet<int> GrowToBoundary(Stack<int> seeds)
    {
        HashSet<int> traversed = new();
        while (seeds.Count > 0)
        {
            int iT = seeds.Pop(); traversed.Add(iT);
            for (int i = 0; i < 3; i++)
            {
                Edge opEdge = new(triangles[iT].vertices[Triangle.CCW(i)], triangles[iT].vertices[Triangle.CW(i)]);
                if (fixedEdges.Contains(opEdge)) continue;
                int iN = triangles[iT].neighbors[Triangle.OppositeNeghbor(i)];
                if (iN != noNeighbor && !traversed.Contains(iN)) seeds.Push(iN);
            }
        }
        return traversed;
    }
    void FixEdge(Edge edge) { if (!fixedEdges.Add(edge)) if (overlapCount.ContainsKey(edge)) ++overlapCount[edge]; else overlapCount[edge] = 1; }
    void FixEdge(Edge edge, Edge originalEdge)
    {
        FixEdge(edge); if (edge != originalEdge) DictionaryInsertUnique(pieceToOriginals, edge, originalEdge);
    }
    static void InsertUnique<T>(ICollection<T> to, T elem) { if (!to.Contains(elem)) to.Add(elem); }
    void InsertUnique<T>(ICollection<T> to, IEnumerable<T> from) { foreach (var item in from) if (!to.Contains(item)) to.Add(item); }
    void DictionaryInsertUnique<T1, T2>(Dictionary<T1, List<T2>> values, T1 key, T2 value)
    { if (values.ContainsKey(key)) InsertUnique(values[key], value); else values[key] = new() { value }; }
    void DictionaryInsertUnique<T1, T2>(Dictionary<T1, List<T2>> values, T1 key, List<T2> value)
    { if (values.ContainsKey(key)) InsertUnique(values[key], value); else values[key] = value; }
    /// <summary>Split existing constraint (fixed) edge</summary>
    /// <param name="edge">fixed edge to split</param>
    /// <param name="iSplitVert">index of the vertex to be used as a split vertex</param>
    void SplitFixedEdge(Edge edge, int iSplitVert)
    {
        Edge half1 = new(edge.iV1, iSplitVert), half2 = new(iSplitVert, edge.iV2);
        fixedEdges.Remove(edge); FixEdge(half1); FixEdge(half2);
        if (overlapCount.TryGetValue(edge, out var overlapIt))
        {
            if (overlapCount.ContainsKey(half1)) overlapCount[half1] += overlapIt; else overlapCount[half1] = overlapIt;
            if (overlapCount.ContainsKey(half2)) overlapCount[half2] += overlapIt; else overlapCount[half2] = overlapIt;
            overlapCount.Remove(edge);
        }
        List<Edge> newOriginals = new() { edge };
        if (pieceToOriginals.TryGetValue(edge, out var originalsIt))
        {
            newOriginals = originalsIt; pieceToOriginals.Remove(edge);
        }
        DictionaryInsertUnique(pieceToOriginals, half1, newOriginals);
        DictionaryInsertUnique(pieceToOriginals, half2, newOriginals);
    }
    /// <summary>Add a vertex that splits an edge into the triangulation</summary>
    /// <param name="splitVert">position of split vertex</param>
    /// <param name="iT">index of a first triangle adjacent to the split edge</param>
    /// <param name="iTopo">index of a second triangle adjacent to the split edge (opposed to the first triangle)</param>
    /// <returns>index of a newly added split vertex</returns>
    int AddSplitEdgeVertex(Vector2 splitVert, int iT, int iTopo)
    {
        int isplitVert = vertices.Count;
        AddNewVertex(splitVert, noNeighbor);
        Stack<int> triStack = InsertVertexOnEdge(isplitVert, iT, iTopo);
        TryAddVertexToLocator(isplitVert);
        EnsureDelaunayByEdgeFlips(isplitVert, triStack);
        return isplitVert;
    }
    /// <summary>Split fixed edge and add a split vertex into the triangulation</summary>
    /// <param name="edge">fixed edge to split</param>
    /// <param name="splitVert">position of split vertex</param>
    /// <param name="iT">index of a first triangle adjacent to the split edge</param>
    /// <param name="iTopo">index of a second triangle adjacent to the split edge (opposed to the first triangle)</param>
    /// <returns>index of a newly added split vertex</returns>
    int SplitFixedEdgeAt(Edge edge, Vector2 splitVert, int iT, int iTopo)
    {
        int iSplitVert = AddSplitEdgeVertex(splitVert, iT, iTopo);
        SplitFixedEdge(edge, iSplitVert); return iSplitVert;
    }
    /// <summary>Flag triangle as dummy</summary>
    /// <param name="iT">index of a triangle to flag</param>
    /// <remarks>Advanced method for manually modifying the triangulation from outside. Please call it when you know what you are doing.</remarks>
    void MakeDummy(int iT) => dummyTris.Add(iT);
    /// <summary>Erase all dummy triangles</summary>
    /// <remarks>Advanced method for manually modifying the triangulation from outside. Please call it when you know what you are doing.</remarks>
    void EraseDummies()
    {
        if (dummyTris.Count == 0) return;
        HashSet<int> dummySet = new(dummyTris);
        Dictionary<int, int> triIndMap = new() { [noNeighbor] = noNeighbor };
        for (int i = 0, itNew = 0; i < triangles.Count; i++)
        {
            if (dummySet.Contains(i)) continue;
            triIndMap[i] = itNew;
            triangles[itNew] = triangles[i];
            itNew++;
        }
        triangles.RemoveRange(dummySet.Count, triangles.Count - dummySet.Count);
        for (int i = 0; i < vertTris.Count; i++) vertTris[i] = triIndMap[vertTris[i]];
        for (int t = 0; t < triangles.Count; t++) for (int i = 0; i < 3; i++)
                triangles[t].neighbors[i] = triIndMap[triangles[t].neighbors[i]];
        dummyTris.Clear();
    }
    /// <summary>Depth-peel a layer in triangulation, used when calculating triangle
    /// depths<br/>
    /// It takes starting seed triangles, traverses neighboring triangles, and
    /// assigns given layer depth to the traversed triangles. Traversal is
    /// blocked by constraint edges. Triangles behind constraint edges are
    /// recorded as seeds of next layer and returned from the function.
    /// </summary>
    /// <param name="seeds">indices of seed triangles</param>
    /// <param name="layerDepth">current layer's depth to mark triangles with</param>
    /// <param name="triDepths">depths of triangles</param>
    /// <returns>triangles of the deeper layers that are adjacent to the peeled layer. To be used as seeds when peeling deeper layers.</returns>
    Dictionary<int, ushort> PeelLayer(Stack<int> seeds, ushort layerDepth, List<ushort> triDepths)
    {
        Dictionary<int, ushort> behindBoundary = new();
        while (seeds.Count > 0)
        {
            int iT = seeds.Pop();
            triDepths[iT] = Math.Min(triDepths[iT], layerDepth);
            behindBoundary.Remove(iT);
            for (int i = 0; i < 3; i++)
            {
                Edge opEdge = new(triangles[iT].vertices[Triangle.CCW(i)], triangles[iT].vertices[Triangle.CW(i)]);
                int iN = triangles[iT].neighbors[Triangle.OppositeNeghbor(i)];
                if (iN == noNeighbor || triDepths[iN] <= layerDepth) continue;
                if (fixedEdges.Contains(opEdge))
                {
                    ushort triDepth = (ushort)(overlapCount.FirstOrDefault(x => x.Key == opEdge) is KeyValuePair<Edge, ushort> e ? layerDepth + e.Value + 1 : layerDepth + 1);
                    behindBoundary[iN] = triDepth; continue;
                }
                seeds.Push(iN);
            }
        }
        return behindBoundary;
    }
    void InsertVertices_AsProvided(int superGeomVertCount)
    {
        for (int iV = superGeomVertCount; iV < vertices.Count; iV++) InsertVertex(iV);
    }
    void InsertVertices_Randomized(int superGeomVertCount)
    {
        static void RandomShuffle<T>(List<T> values)
        {
            int n = values.Count; SplitMix64RandGen prng = new();
            for (int i = n - 1; i > 0; i--) { int j = (int)(prng.Next() % ((ulong)i + 1)); (values[i], values[j]) = (values[j], values[i]); }
        }
        int vertexCount = vertices.Count * superGeomVertCount;
        List<int> ii = new(vertexCount);
        for (int i = 0; i < vertexCount; i++) ii.Add(i + superGeomVertCount);
        RandomShuffle(ii);
        for (int it = 0; it < ii.Count; it++) InsertVertex(ii[it]);
    }
    void InsertVertices_KDTreeBFS(int superGeomVertCount, Vector2 boxMin, Vector2 boxMax)
    {
        static int MaxQueueLengthBSKDTree(int vertexCount)
        {
            int filledLayerPow2 = (int)Math.Floor(Math.Log2(vertexCount)) - 1,
                nodesInFilledTree = (int)Math.Pow(2, filledLayerPow2 + 1) - 1,
                nodesInLastFilledLayer = (int)Math.Pow(2, filledLayerPow2),
                nodesInLastLayer = vertexCount - nodesInFilledTree;
            return nodesInLastLayer >= nodesInLastFilledLayer ? nodesInLastFilledLayer + nodesInLastLayer - nodesInLastFilledLayer : nodesInLastFilledLayer;
        }
        int vertexCount = vertices.Count - superGeomVertCount; if (vertexCount <= 0) return;
        List<int> ii = new(vertexCount);
        for (int i = 0; i < vertexCount; i++) ii.Add(i + superGeomVertCount);
        Queue<(int, int, Vector2, Vector2, int)> queue = new(MaxQueueLengthBSKDTree(vertexCount));
        queue.Enqueue((0, ii.Count, boxMin, boxMax, 0));
        Vector2 newBoxMin, newBoxMax; int first, last, parent, mid;
        while (queue.Count > 0)
        {
            (first, last, boxMin, boxMax, parent) = queue.Dequeue();
            Debug.Assert(first != last);
            int len = last - first;
            if (len == 1) { InsertVertex(ii[first], parent); continue; }
            int midIt = first + len / 2;
            if (boxMax.X - boxMin.X >= boxMax.Y - boxMin.Y)
            {
                Triangle.NthElement(ii, first, midIt, last, (v1, v2) => vertices[v1].X.CompareTo(vertices[v2].X));
                mid = ii[midIt];
                double split = vertices[mid].X;
                newBoxMin = new(split, boxMin.Y);
                newBoxMax = new(split, boxMax.Y);
            }
            else
            {
                Triangle.NthElement(ii, first, midIt, last, (v1, v2) => vertices[v1].Y.CompareTo(vertices[v2].Y));
                mid = ii[midIt];
                double split = vertices[mid].Y;
                newBoxMin = new(boxMin.X, split);
                newBoxMax = new(boxMax.X, split);
            }
            InsertVertex(mid, parent);
            if (first != midIt) queue.Enqueue((first, midIt, boxMin, newBoxMax, mid));
            if (midIt + 1 != last) queue.Enqueue((midIt + 1, last, newBoxMin, boxMax, mid));
        }
    }
    (int, int) EdgeTriangles(int a, int b)
    {
        int triStart = vertTris[a];
        Debug.Assert(triStart != noNeighbor);
        int iT = triStart, iTNext, iV;
        do
        {
            (iTNext, iV) = triangles[iT].Next(a);
            Debug.Assert(iTNext != noNeighbor);
            if (iV == b) return (iT, iTNext);
            iT = iTNext;
        } while (iT!=triStart);
        return (invalidIndex, invalidIndex);
    }
    bool HasEdge(int a, int b) => EdgeTriangles(a, b).Item1 != invalidIndex;
    void SetAdjancentTriangle(int v, int t)
    {
        Debug.Assert(t != noNeighbor); vertTris[v] = t;
        Debug.Assert(triangles[t].vertices[0] == v || triangles[t].vertices[1] == v || triangles[t].vertices[2] == v);
    }
    void PivotVertexTriangleCW(int v)
    {
        Debug.Assert(vertTris[v] != noNeighbor);
        vertTris[v] = triangles[vertTris[v]].Next(v).Item1;
        Debug.Assert(vertTris[v] != noNeighbor);
        Debug.Assert(triangles[vertTris[v]].vertices[0] == v || triangles[vertTris[v]].vertices[1] == v || triangles[vertTris[v]].vertices[2] == v);
    }
    void TryAddVertexToLocator(int v) { if (nearPtLocator.Size == 0) nearPtLocator.Insert(v, vertices); }
    void TryInitNearestPointLocator() { if (vertices.Count == 0 && nearPtLocator.Size == 0) nearPtLocator = new(vertices); }
    List<int> dummyTris = new();
    KDTree nearPtLocator = new();
    int nTargetVerts;
    public SuperGeometryType superGeomType;
    VertexInsertionOrder vertexInsertionOrder;
    IntersectingConstraintEdges intersectingEdgesStrategy;
    double minDistToConstraintEdge;
    List<int> vertTris = new();
}
