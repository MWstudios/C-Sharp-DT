using System;
using System.Diagnostics;
using System.Collections.Generic;
using System.Linq;

namespace CDT;

public enum VertexInsertionOrder { Auto, AsProvided }
public enum SuperGeometryType { SuperTriangle, Custom }
public enum IntersectingConstraintEdges { NotAllowed, TryResolve, DontCheck }
public enum PtTriLocation { Inside, Outside, OnEdge1, OnEdge2, OnEdge3, OnVertex }
public enum PtLineLocation { Left, Right, OnLine }
public enum RefinementCriterion { SmallestAngle, LargestArea }
public struct Box
{
    public Vector2 min, max;
    public void EnvelopPoint(Vector2 p) => EnvelopPoint(p.X, p.Y);
    public void EnvelopPoint(double x, double y)
    {
        min.X = Math.Min(x, min.X); max.X = Math.Max(x, max.X);
        min.Y = Math.Min(y, min.Y); max.Y = Math.Max(y, max.Y);
    }
    public static Box EnvelopBox(Span<Vector2> vertices)
    {
        Box box = new() { min = new(double.PositiveInfinity), max = new(double.NegativeInfinity) };
        for (int i = 0; i < vertices.Length; i++) box.EnvelopPoint(vertices[i]); return box;
    }
    public static Box EnvelopBox(IEnumerable<Vector2> vertices)
    {
        Box box = new() { min = new(double.PositiveInfinity), max = new(double.NegativeInfinity) };
        foreach (var v in vertices) box.EnvelopPoint(v); return box;
    }
}
[DebuggerDisplay("{iV1} {iV2}")]
public class Edge
{
    public int iV1, iV2;
    public Edge(int iV1, int iV2) { if (iV1 > iV2) (iV1, iV2) = (iV2, iV1); this.iV1 = iV1; this.iV2 = iV2; }
    public static bool operator ==(Edge a, Edge b) => a.iV1 == b.iV1 && a.iV2 == b.iV2;
    public static bool operator !=(Edge a, Edge b) => a.iV1 != b.iV1 || a.iV2 != b.iV2;
    public override int GetHashCode()
    {
        void HashCombine(ref int seed, int key) { seed ^= HashCode.Combine(key) + unchecked((int)0x9e3779b9) + (seed << 6) + (seed >> 2); }
        int seed1 = 0; HashCombine(ref seed1, iV1); HashCombine(ref seed1, iV2);
        int seed2 = 0; HashCombine(ref seed1, iV2); HashCombine(ref seed1, iV1);
        return Math.Min(seed1, seed2);
    }
    public override bool Equals(object obj) => obj is Edge e && this == e;
}
[DebuggerDisplay("V {vertices[0]} {vertices[1]} {vertices[2]}, N {neighbors[0]} {neighbors[1]} {neighbors[2]}")]
public class Triangle
{
    public static void NthElement<T>(List<T> values, int first, int nth, int last, Comparison<T> comp)
    {
        static int sort3(List<T> values, int x, int y, int z, Comparison<T> c)
        {
            int r = 0;
            if (c(values[y], values[x]) == 1) // if x <= y
            {
                if (c(values[z], values[y]) == 1)     // if y <= z
                    return r;      // x <= y && y <= z
                                   // x <= y && y > z
                (values[y], values[z]) = (values[z], values[y]); // x <= z && y < z
                r = 1;
                if (c(values[y], values[x]) == -1) // if x > y
                {
                    (values[x], values[y]) = (values[y], values[x]); // x < y && y <= z
                    r = 2;
                }
                return r; // x <= y && y < z
            }
            if (c(values[z], values[y]) == -1) // x > y, if y > z
            {
                (values[x], values[z]) = (values[z], values[x]); // x < y && y < z
                r = 1;
                return r;
            }
            (values[x], values[y]) = (values[y], values[x]); // x > y && y <= z
            r = 1;             // x < y && x <= z
            if (c(values[z], values[y]) == -1)      // if y > z
            {
                (values[y], values[z]) = (values[z], values[y]); // x <= y && y < z
                r = 2;
            }
            return r;
        } // x <= y && y <= z
        // Compare is known to be a reference type
        int limit = 7;
        while (true)
        {
        restart:
            if (nth == last)
                return;
            int len = last - first;
            switch (len)
            {
                case 0:
                case 1:
                    return;
                case 2:
                    if (comp(values[--last], values[first]) == -1)
                        (values[first], values[last]) = (values[last], values[first]);
                    return;
                case 3:
                    {
                        int m1 = first;
                        sort3(values, first, ++m1, --last, comp);
                        return;
                    }
            }
            if (len <= limit)
            {
                values.Sort(first, last - first, Comparer<T>.Create(comp));
                return;
            }
            // len > limit >= 3
            int m = first + len / 2;
            int lm1 = last;
            int n_swaps = sort3(values, first, m, --lm1, comp);
            // *m is median
            // partition [first, m) < *m and *m <= [m, last)
            // (this inhibits tossing elements equivalent to m around
            // unnecessarily)
            int i = first;
            int j = lm1;
            // j points beyond range to be tested, *lm1 is known to be <= *m
            // The search going up is known to be guarded but the search coming
            // down isn't. Prime the downward search with a guard.
            if (comp(values[i], values[m]) == 1) // if *first == *m
            {
                // *first == *m, *first doesn't go in first part
                // manually guard downward moving j against i
                while (true)
                {
                    if (i == --j)
                    {
                        // *first == *m, *m <= all other elements
                        // Parition instead into [first, i) == *first and
                        // *first < [i, last)
                        ++i; // first + 1
                        j = last;
                        if (comp(values[first], values[--j]) == 1) // we need a guard if
                                                 // *first == *(last-1)
                        {
                            while (true)
                            {
                                if (i == j)
                                    return; // [first, last) all equivalent
                                            // elements
                                if (comp(values[first], values[i]) == -1)
                                {
                                    (values[i], values[j]) = (values[j], values[i]);
                                    ++n_swaps;
                                    ++i;
                                    break;
                                }
                                ++i;
                            }
                        }
                        // [first, i) == *first and *first < [j,
                        // last) and j == last - 1
                        if (i == j)
                            return;
                        while (true)
                        {
                            while (comp(values[first], values[i]) == 1)
                                ++i;
                            while (comp(values[first], values[--j]) == -1)
                                ;
                            if (i >= j)
                                break;
                            (values[i], values[j]) = (values[j], values[i]);
                            ++n_swaps;
                            ++i;
                        }
                        // [first, i) == *first and *first < [i,
                        // last) The first part is sorted,
                        if (nth < i)
                            return;
                        // nth_element the secod part
                        // nth_element<Compare>(i, nth, last, comp);
                        first = i;
                        goto restart;
                    }
                    if (comp(values[j], values[m]) == -1)
                    {
                        (values[i], values[j]) = (values[j], values[i]);
                        ++n_swaps;
                        break; // found guard for downward moving j, now use
                               // unguarded partition
                    }
                }
            }
            ++i;
            // j points beyond range to be tested, *lm1 is known to be <= *m
            // if not yet partitioned...
            if (i < j)
            {
                // known that *(i - 1) < *m
                while (true)
                {
                    // m still guards upward moving i
                    while (comp(values[i], values[m]) == -1)
                        ++i;
                    // It is now known that a guard exists for downward moving
                    // j
                    while (comp(values[--j], values[m]) == 1)
                        ;
                    if (i >= j)
                        break;
                    (values[i], values[j]) = (values[j], values[i]);
                    ++n_swaps;
                    // It is known that m != j
                    // If m just moved, follow it
                    if (m == i)
                        m = j;
                    ++i;
                }
            }
            // [first, i) < *m and *m <= [i, last)
            if (i != m && comp(values[m], values[i]) == -1)
            {
                (values[i], values[m]) = (values[m], values[i]);
                ++n_swaps;
            }
            // [first, i) < *i and *i <= [i+1, last)
            if (nth == i)
                return;
            if (n_swaps == 0)
            {
                // We were given a perfectly partitioned sequence.  Coincidence?
                if (nth < i)
                {
                    // Check for [first, i) already sorted
                    j = m = first;
                    while (++j != i)
                    {
                        if (comp(values[j], values[m]) == -1)
                            // not yet sorted, so sort
                            goto not_sorted;
                        m = j;
                    }
                    // [first, i) sorted
                    return;
                }
                else
                {
                    // Check for [i, last) already sorted
                    j = m = i;
                    while (++j != last)
                    {
                        if (comp(values[j], values[m]) == -1)
                            // not yet sorted, so sort
                            goto not_sorted;
                        m = j;
                    }
                    // [i, last) sorted
                    return;
                }
            }
        not_sorted:
            // nth_element on range containing nth
            if (nth < i)
            {
                // nth_element<Compare>(first, nth, i, comp);
                last = i;
            }
            else
            {
                // nth_element<Compare>(i+1, nth, last, comp);
                first = ++i;
            }
        }
    }
    struct Expansion
    {
        double[] values; int m_size = 0;
        public Expansion(int size) { values = new double[size]; }
        public double mostSignificant() => m_size == 0 ? 0 : values[m_size - 1];
        public void push_back(double value) => values[m_size++] = value;
        public double estimate() => values.Take(m_size).Sum();
        public static Expansion operator -(Expansion a)
        {
            a = new(a.values.Length) { m_size = a.m_size };
            for (int i = 0; i < a.values.Length; i++) a.values[i] = -a.values[i];
            return a;
        }
        public static Expansion operator +(Expansion a, Expansion b)
        {
            static double[] merge(double[] e, double[] f, Func<double, double, bool> comp) //std::merge
            {
                double[] h = new double[e.Length + f.Length]; int first2 = 0, d_first = 0;
                for (int first1 = 0; first1 != e.Length; ++d_first)
                {
                    if (first2 == f.Length) { Array.Copy(e, first1, h, d_first, e.Length - first1); return h; }
                    if (comp(f[first2], e[first1]))
                    {
                        h[d_first] = f[first2];
                        ++first2;
                    }
                    else
                    {
                        h[d_first] = e[first1];
                        ++first1;
                    }
                }
                Array.Copy(f, first2, h, d_first, f.Length - first2); return h;
            }
            static int ExpansionSum(double[] e, int n, double[] f, int m, out double[] h)
            {
                h = merge(e, f, (a, b) => Math.Abs(a) < Math.Abs(b));
                if (m == 0) return n;
                if (n == 0) return m;
                int hIndex = 0;
                double Q = h[0];
                double Qnew = h[1] + Q;
                double hh = FastPlusTail(h[1], Q, Qnew);
                Q = Qnew;
                if (0 != hh) h[hIndex++] = hh;
                for (int g = 2; g != n + m; ++g)
                {
                    Qnew = Q + h[g];
                    hh = PlusTail(Q, h[g], Qnew);
                    Q = Qnew;
                    if (0 != hh) h[hIndex++] = hh;
                }
                if (0 != Q) h[hIndex++] = Q;
                return hIndex;
            }
            Expansion h = new(a.values.Length + b.values.Length);
            h.m_size = ExpansionSum(a.values, a.m_size, b.values, b.m_size, out h.values);
            return h;
        }
        public static Expansion operator -(Expansion a, Expansion b) => a + -b;
        public static Expansion operator *(Expansion a, double b)
        {
            //scale an expansion by a constant
            static int ScaleExpansion(double[] e, int n, double b, double[] h)
            {
                //static (double, double) Split(double a) { double c = a * Math.ScaleB(1, 52 / 2 + 1), aBig = c - a, aHi = c - aBig; return (aHi, a - aHi); }
                if (n == 0 || 0 == b) return 0;
                int hIndex = 0;
                double Q = e[0] * b;
                double hh = MultTail(e[0], b, Q);
                if (0 != hh) h[hIndex++] = hh;
                for (int eIndex = 1; eIndex < n; ++eIndex)
                {
                    double Ti = e[eIndex] * b;
                    double ti = MultTail(e[eIndex], b, Ti);
                    double Qi = Q + ti;
                    hh = PlusTail(Q, ti, Qi);
                    if (0 != hh) h[hIndex++] = hh;
                    Q = Ti + Qi;
                    hh = FastPlusTail(Ti, Qi, Q);
                    if (0 != hh) h[hIndex++] = hh;
                }
                if (0 != Q) h[hIndex++] = Q;
                return hIndex;
            }
            Expansion h = new(a.values.Length * 2);
            h.m_size = ScaleExpansion(a.values, a.m_size, b, h.values);
            return h;
        }
    }
    static double PlusTail(double a, double b, double x)
    { double bVirtual = x - a, aVirtual = x - bVirtual, bRoundoff = b - bVirtual, aRoundoff = a - aVirtual; return aRoundoff + bRoundoff; }
    static double FastPlusTail(double a, double b, double x) { double bVirtual = x - a; return b - bVirtual; }
    static double MinusTail(double a, double b, double x)
    { double bVirtual = a - x, aVirtual = x + bVirtual, bRoundoff = bVirtual - b, aRoundoff = a - aVirtual; return aRoundoff + bRoundoff; }
    static double MultTail(double a, double b, double p) => Math.FusedMultiplyAdd(a, b, -p);
    static Expansion TwoTwoDiff(double ax, double by, double ay, double bx)
    {
        double axby1 = ax * by;
        double axby0 = MultTail(ax, by, axby1);
        double bxay1 = bx * ay;
        double bxay0 = MultTail(bx, ay, bxay1);
        double _i0 = axby0 - bxay0;
        double x0 = MinusTail(axby0, bxay0, _i0);
        double _j = axby1 + _i0;
        double _0 = PlusTail(axby1, _i0, _j);
        double _i1 = _0 - bxay1;
        double x1 = MinusTail(_0, bxay1, _i1);
        double x3 = _j + _i1;
        double x2 = PlusTail(_j, _i1, x3);
        Expansion e = new(4);
        if (0 != x0) e.push_back(x0);
        if (0 != x1) e.push_back(x1);
        if (0 != x2) e.push_back(x2);
        if (0 != x3) e.push_back(x3);
        return e;
    }
    static double resulterrbound = ( 3 +   8 * Math.ScaleB(1, -52)) * Math.ScaleB(1, -52);
    static double ccwerrboundA   = ( 3 +  16 * Math.ScaleB(1, -52)) * Math.ScaleB(1, -52);
    static double ccwerrboundB   = ( 2 +  12 * Math.ScaleB(1, -52)) * Math.ScaleB(1, -52);
    static double ccwerrboundC   = ( 9 +  64 * Math.ScaleB(1, -52)) * Math.ScaleB(1, -52) * Math.ScaleB(1, -52);
    static double iccerrboundA   = (10 +  96 * Math.ScaleB(1, -52)) * Math.ScaleB(1, -52);
    static double iccerrboundB   = ( 4 +  48 * Math.ScaleB(1, -52)) * Math.ScaleB(1, -52);
    static double iccerrboundC   = (44 + 576 * Math.ScaleB(1, -52)) * Math.ScaleB(1, -52) * Math.ScaleB(1, -52);
    public static double Orient2D(Vector2 p, Vector2 v1, Vector2 v2)
    {
        Expansion aterms = TwoTwoDiff(v1.X, v2.Y, v1.X, p.Y);
        Expansion bterms = TwoTwoDiff(v2.X, p.Y, v2.X, v1.Y);
        Expansion cterms = TwoTwoDiff(p.X, v1.Y, p.Y, v2.Y);
        Expansion w = aterms + bterms + cterms;
        return w.mostSignificant();
    }
    public static double Orient2D(double ax, double ay, double bx, double by, double cx, double cy)
    {
        double acx = ax - cx;
        double bcx = bx - cx;
        double acy = ay - cy;
        double bcy = by - cy;
        double detleft = acx * bcy;
        double detright = acy * bcx;
        double det = detleft - detright;
        if ((detleft < 0) != (detright < 0)) return det;
        if (0 == detleft || 0 == detright) return det;

        double detsum = Math.Abs(detleft + detright);
        double errbound = ccwerrboundA * detsum;
        if (Math.Abs(det) >= Math.Abs(errbound)) return det;

        Expansion B = TwoTwoDiff(acx, bcy, acy, bcx);
        det = B.estimate();
        errbound = ccwerrboundB * detsum;
        if (Math.Abs(det) >= Math.Abs(errbound)) return det;

        double acxtail = MinusTail(ax, cx, acx);
        double bcxtail = MinusTail(bx, cx, bcx);
        double acytail = MinusTail(ay, cy, acy);
        double bcytail = MinusTail(by, cy, bcy);
        if (0 == acxtail && 0 == bcxtail && 0 == acytail && 0 == bcytail) return det;

        errbound = ccwerrboundC * detsum + resulterrbound * Math.Abs(det);
        det += (acx * bcytail + bcy * acxtail) - (acy * bcxtail + bcx * acytail);
        if (Math.Abs(det) >= Math.Abs(errbound)) return det;

        Expansion D = B + TwoTwoDiff(acxtail, bcy, acytail, bcx) + TwoTwoDiff(acx, bcytail, acy, bcxtail) + TwoTwoDiff(acxtail, bcytail, acytail, bcxtail);
        return D.mostSignificant();
    }
    static double InCircle(double ax, double ay, double bx, double by, double cx, double cy, double dx, double dy)
    {
        Expansion ab = TwoTwoDiff(ax, by, bx, ay);
        Expansion bc = TwoTwoDiff(bx, cy, cx, by);
        Expansion cd = TwoTwoDiff(cx, dy, dx, cy);
        Expansion da = TwoTwoDiff(dx, ay, ax, dy);
        Expansion ac = TwoTwoDiff(ax, cy, cx, ay);
        Expansion bd = TwoTwoDiff(bx, dy, dx, by);

        Expansion abc = ab + bc - ac;
        Expansion bcd = bc + cd - bd;
        Expansion cda = cd + da + ac;
        Expansion dab = da + ab + bd;

        Expansion adet = bcd * ax * ax + bcd * ay * ay;
        Expansion bdet = cda * bx * -bx + cda * by * -by;
        Expansion cdet = dab * cx * cx + dab * cy * cy;
        Expansion ddet = abc * dx * -dx + abc * dy * -dy;

        Expansion deter = adet + bdet + (cdet + ddet);
        return deter.mostSignificant();
    }
    public static bool IsInCircumcircle(Vector2 p, Vector2 v1, Vector2 v2, Vector2 v3)
    {
        double adx = v1.X - p.X;
        double bdx = v2.X - p.X;
        double cdx = v3.X - p.X;
        double ady = v1.Y - p.Y;
        double bdy = v2.Y - p.Y;
        double cdy = v3.Y - p.Y;
        double bdxcdy = bdx * cdy;
        double cdxbdy = cdx * bdy;
        double cdxady = cdx * ady;
        double adxcdy = adx * cdy;
        double adxbdy = adx * bdy;
        double bdxady = bdx * ady;
        double alift = adx * adx + ady * ady;
        double blift = bdx * bdx + bdy * bdy;
        double clift = cdx * cdx + cdy * cdy;
        double det = alift * (bdxcdy - cdxbdy) + blift * (cdxady - adxcdy) + clift * (adxbdy - bdxady);
        double permanent = (Math.Abs(bdxcdy) + Math.Abs(cdxbdy)) * alift
                          + (Math.Abs(cdxady) + Math.Abs(adxcdy)) * blift
                          + (Math.Abs(adxbdy) + Math.Abs(bdxady)) * clift;
        double errbound = iccerrboundA * permanent;
        if (Math.Abs(det) >= Math.Abs(errbound)) return det > 0;

        Expansion bc = TwoTwoDiff(bdx, cdy, cdx, bdy);
        Expansion ca = TwoTwoDiff(cdx, ady, adx, cdy);
        Expansion ab = TwoTwoDiff(adx, bdy, bdx, ady);
        Expansion adet = bc * adx * adx + bc * ady * ady;
        Expansion bdet = ca * bdx * bdx + ca * bdy * bdy;
        Expansion cdet = ab * cdx * cdx + ab * cdy * cdy;
        Expansion fin1 = adet + bdet + cdet;
        det = fin1.estimate();
        errbound = iccerrboundB * permanent;
        if (Math.Abs(det) >= Math.Abs(errbound)) return det > 0;

        double adxtail = MinusTail(v1.X, p.X, adx);
        double adytail = MinusTail(v2.X, p.X, ady);
        double bdxtail = MinusTail(v3.X, p.X, bdx);
        double bdytail = MinusTail(v1.Y, p.Y, bdy);
        double cdxtail = MinusTail(v2.Y, p.Y, cdx);
        double cdytail = MinusTail(v3.Y, p.Y, cdy);
        if (0 == adxtail && 0 == bdxtail && 0 == cdxtail && 0 == adytail && 0 == bdytail && 0 == cdytail) return det > 0;

        errbound = iccerrboundC * permanent + resulterrbound * Math.Abs(det);
        det += (adx * adx + ady * ady) * (bdx * cdytail + cdy * bdxtail - (bdy * cdxtail + cdx * bdytail))
            + (bdx * cdy - bdy * cdx) * (adx * adxtail + ady * adytail) * 2
            + ((bdx * bdx + bdy * bdy) * (cdx * adytail + ady * cdxtail - (cdy * adxtail + adx * cdytail))
            + (cdx * ady - cdy * adx) * (bdx * bdxtail + bdy * bdytail) * 2)
            + ((cdx * cdx + cdy * cdy) * (adx * bdytail + bdy * adxtail - (ady * bdxtail + bdx * adytail))
            + (adx * bdy - ady * bdx) * (cdx * cdxtail + cdy * cdytail) * 2);
        if (Math.Abs(det) >= Math.Abs(errbound)) return det > 0;
        return InCircle(v1.X, v1.Y, v2.X, v2.Y, v3.X, v3.Y, p.X, p.Y) > 0;
    }
    //public static double Orient2D(double ax, double ay, double bx, double by, double cx, double cy) => (ax - bx) * (by - cy) + (ay - by) * (cx - bx);
    public int[] vertices = new int[3], neighbors = new int[3];
    public (int, int) Next(int i)
    {
        Debug.Assert(vertices[0] == i || vertices[1] == i || vertices[2] == i);
        return vertices[0] == i ? (neighbors[0], vertices[1]) : vertices[1] == i ? (neighbors[1], vertices[2]) : (neighbors[2], vertices[0]);
    }
    public (int, int) Prev(int i)
    {
        Debug.Assert(vertices[0] == i || vertices[1] == i || vertices[2] == i);
        return vertices[0] == i ? (neighbors[2], vertices[2]) : vertices[1] == i ? (neighbors[0], vertices[0]) : (neighbors[1], vertices[1]);
    }
    public bool ContainsVertex(int i) => vertices[0] == i || vertices[1] == i || vertices[2] == i;
    public static int CCW(int i) => (i + 1) % 3;
    public static int CW(int i) => (i + 2) % 3;
    public static bool IsOnEdge(PtTriLocation location) => location == PtTriLocation.OnEdge1 || location == PtTriLocation.OnEdge2 || location == PtTriLocation.OnEdge3;
    public static int EdgeNeighbor(PtTriLocation location)
    { Debug.Assert(IsOnEdge(location)); return location - PtTriLocation.OnEdge1; }
    public static PtLineLocation LocatePointLine(Vector2 p, Vector2 v1, Vector2 v2, double orientationTolerance = 0)
        => ClassifyOrientation(Orient2D(v1.X, v1.Y, v2.X, v2.Y, p.X, p.Y), orientationTolerance);
    public static PtLineLocation ClassifyOrientation(double orientation, double orientationTolerance = 0) =>
        orientation < -orientationTolerance ? PtLineLocation.Right : orientation > orientationTolerance ? PtLineLocation.Left : PtLineLocation.OnLine;
    public static PtTriLocation LocatePointTriangle(Vector2 p, Vector2 v1, Vector2 v2, Vector2 v3)
    {
        PtTriLocation result = PtTriLocation.Inside;
        PtLineLocation edgeCheck = LocatePointLine(p, v1, v2);
        if (edgeCheck == PtLineLocation.Right) return PtTriLocation.Outside;
        if (edgeCheck == PtLineLocation.OnLine) result = PtTriLocation.OnEdge1;
        edgeCheck = LocatePointLine(p, v2, v3);
        if (edgeCheck == PtLineLocation.Right) return PtTriLocation.Outside;
        if (edgeCheck == PtLineLocation.OnLine) result = result == PtTriLocation.Inside ? PtTriLocation.OnEdge2 : PtTriLocation.OnVertex;
        edgeCheck = LocatePointLine(p, v3, v1);
        if (edgeCheck == PtLineLocation.Right) return PtTriLocation.Outside;
        if (edgeCheck == PtLineLocation.OnLine) result = result == PtTriLocation.Inside ? PtTriLocation.OnEdge3 : PtTriLocation.OnVertex;
        return result;
    }
    public static int OppositeNeghbor(int vertIndex) => vertIndex >= 0 && vertIndex < 3 ? (vertIndex + 1) % 3 : throw new ArgumentOutOfRangeException("Invalid vertex index");
    public static int OppositeVertex(int neighborIndex) => neighborIndex >= 0 && neighborIndex < 3 ? (neighborIndex + 2) % 3 : throw new ArgumentOutOfRangeException("Invalid neighbor index");
    public static int OpposedTriangleIndex(int[] vv, int index)
    {
        Debug.Assert(vv[0] == index || vv[1] == index || vv[2] == index);
        return vv[0] == index ? 1 : vv[1] == index ? 2 : 0;
    }
    public static int EdgeNeighborIndex(int[] vv, int iVedge1, int iVedge2)
    {
        Debug.Assert(vv[0] == iVedge1 || vv[1] == iVedge1 || vv[2] == iVedge1);
        Debug.Assert(vv[0] == iVedge2 || vv[1] == iVedge2 || vv[2] == iVedge2);
        Debug.Assert((vv[0] != iVedge1 && vv[0] != iVedge2) || (vv[1] != iVedge1 && vv[1] != iVedge2) || (vv[2] != iVedge1 && vv[2] != iVedge2));
        if (vv[0] == iVedge1) return vv[1] == iVedge2 ? 0 : 2;
        if (vv[0] == iVedge2) return vv[1] == iVedge1 ? 0 : 2;
        return 1;
    }
    public static int OpposedVertexIndex(int[] nn, int iTopo)
    {
        Debug.Assert(nn[0] == iTopo || nn[1] == iTopo || nn[2] == iTopo);
        return nn[0] == iTopo ? 2 : nn[1] == iTopo ? 0 : 1;
    }
    public static int VertexIndex(int[] vv, int iV)
    {
        Debug.Assert(vv[0] == iV || vv[1] == iV || vv[2] == iV);
        return vv[0] == iV ? 0 : vv[1] == iV ? 1 : 2;
    }
    public static int OpposedTriangle(Triangle tri, int iVert) => tri.neighbors[OpposedTriangleIndex(tri.vertices, iVert)];
    public static int EdgeNeighbor(Triangle tri, int iVedge1, int iVedge2) => tri.neighbors[EdgeNeighborIndex(tri.vertices, iVedge1, iVedge2)];
    public static int OpposedVertex(Triangle tri, int iTopo) => tri.vertices[OpposedVertexIndex(tri.neighbors, iTopo)];
    
    public static bool VerticesShareEdge(List<int> aTris, List<int> bTris)
    {
        for (int i = 0; i < aTris.Count; i++) if (bTris.Contains(aTris[i])) return true;
        return false;
    }
    public static bool TouchesSuperTriangle(Triangle t) => t.vertices[0] < 3 || t.vertices[1] < 3 || t.vertices[2] < 3;
    public static bool IsEncroachingOnEdge(Vector2 v, Vector2 edgeStart, Vector2 edgeEnd) => (edgeStart.X - v.X) * (edgeEnd.X - v.X) + (edgeStart.Y - v.Y) * (edgeEnd.Y - v.Y) <= 0;
    public static Vector2 Circumcenter(Vector2 a, Vector2 b, Vector2 c)
    {
        double denom = 2 * Orient2D(a, b, c);
        Debug.Assert(denom != 0);
        a -= c; b -= c;
        double aLenSq = a.LengthSquared(), bLenSq = b.LengthSquared();
        c.X += (b.Y * aLenSq - a.Y * bLenSq) / denom;
        c.Y += (a.X * bLenSq - b.X * aLenSq) / denom;
        return c;
    }
    public static double DoubledArea(Vector2 a, Vector2 b, Vector2 c) => Math.Abs(Orient2D(a, b, c));
    public static double Area(Vector2 a, Vector2 b, Vector2 c) => DoubledArea(a, b, c) / 2;
    public static double SineOfSmallestAngle(Vector2 a, Vector2 b, Vector2 c)
    {
        double sideA = Vector2.Distance(a, b), sideB = Vector2.Distance(b, c);
        if (sideA > sideB) (sideA, sideB) = (sideB, sideA);
        sideA = Math.Max(sideA, Vector2.Distance(a, c));
        return (DoubledArea(a, b, c) / sideA) / sideB;
    }
    public static double SmallestAngle(Vector2 a, Vector2 b, Vector2 c)
    {
        double angleSine = SineOfSmallestAngle(a, b, c);
        Debug.Assert(angleSine >= -1 && angleSine <= 1);
        return Math.Asin(angleSine);
    }
}
struct SplitMix64RandGen
{
    ulong m_state = 0;
    public SplitMix64RandGen(ulong state) => m_state = state;
    public SplitMix64RandGen() { }
    public ulong Next()
    {
        ulong z = m_state += 0x9e3779b97f4a7c15;
        z = (z ^ (z >> 30)) * 0xbf58476d1ce4e5b9;
        z = (z ^ (z >> 27)) * 0x94d049bb133111eb;
        return z ^ (z >> 31);
    }
}