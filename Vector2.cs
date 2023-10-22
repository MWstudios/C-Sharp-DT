using System;

namespace CDT;

public struct Vector2 : IEquatable<Vector2>
{
    private static readonly Vector2 _Zero = new(0, 0), _One = new(1, 1);
    public double X, Y;
    public static Vector2 Zero => _Zero;
    public static Vector2 One => _One;
    public Vector2(double x, double y) { X = x; Y = y; }
    public Vector2(double xy) { X = xy; Y = xy; }
    public static double Dot(Vector2 left, Vector2 right) => left.X * right.X + left.Y * right.Y;
    public static double Distance(Vector2 v1, Vector2 v2)
    {
        double dX = v1.X - v2.X;
        double dY = v1.Y - v2.Y;
        return Math.Sqrt(dX * dX + dY * dY);
    }
    public static double DistanceSquared(Vector2 v1, Vector2 v2)
    {
        double dX = v1.X - v2.X;
        double dY = v1.Y - v2.Y;
        return dX * dX + dY * dY;
    }
    public double Length() => Math.Sqrt(X * X + Y * Y);
    public double LengthSquared() => X * X + Y * Y;
    public void Normalize()
    {
        var length = (double)Math.Sqrt((X * X) + (Y * Y));
        var invLength = 1.0f / length;
        X *= invLength;
        Y *= invLength;
    }
    public Vector2 Skew() => new(-Y, X);
    public override int GetHashCode() => X.GetHashCode() ^ Y.GetHashCode();
    public override bool Equals(object obj) => obj is Vector2 v && this == v;
    #region Implement IEquatable<Vector2>
    public bool Equals(Vector2 other) => X == other.X && Y == other.Y;
    #endregion Implement IEquatable<Vector2>
    public static Vector2 operator +(Vector2 left, Vector2 right) => new(left.X + right.X, left.Y + right.Y);
    public static Vector2 operator +(Vector2 left, double right) => new(left.X + right, left.Y + right);
    public static Vector2 operator +(double left, Vector2 right) => new(left + right.X, left + right.Y);
    public static Vector2 operator -(Vector2 left, Vector2 right) => new(left.X - right.X, left.Y - right.Y);
    public static Vector2 operator -(Vector2 left, double right) => new(left.X - right, left.Y - right);
    public static Vector2 operator -(Vector2 right) => new(-right.X, -right.Y);
    public static Vector2 operator *(Vector2 left, Vector2 right) => new(left.X * right.X, left.Y * right.Y);
    public static Vector2 operator *(Vector2 left, double right) => new(left.X * right, left.Y * right);
    public static Vector2 operator *(double left, Vector2 right) => new(left * right.X, left * right.Y);
    public static Vector2 operator /(Vector2 left, double right) { double invRight = 1 / right; return new(left.X * invRight, left.Y *= invRight); }
    public static bool operator ==(Vector2 left, Vector2 right) => left.X == right.X && left.Y == right.Y;
    public static bool operator !=(Vector2 left, Vector2 right) => left.X != right.X || left.Y != right.Y;
    public override string ToString() => string.Format("{{X: {0} Y: {1}}}", X, Y);
    #region Fast ref methods
    public static void Dot(ref Vector2 left, ref Vector2 right, out double result) => result = left.X * right.X + left.Y * right.Y;
    public static void Min(ref Vector2 v1, ref Vector2 v2, out Vector2 result)
    {
        result.X = (v1.X < v2.X) ? v1.X : v2.X;
        result.Y = (v1.Y < v2.Y) ? v1.Y : v2.Y;
    }
    public static void Max(ref Vector2 v1, ref Vector2 v2, out Vector2 result)
    {
        result.X = (v1.X > v2.X) ? v1.X : v2.X;
        result.Y = (v1.Y > v2.Y) ? v1.Y : v2.Y;
    }
    public static Vector2 Min(Vector2 v1, Vector2 v2) => new((v1.X < v2.X) ? v1.X : v2.X, (v1.Y < v2.Y) ? v1.Y : v2.Y);
    public static Vector2 Max(Vector2 v1, Vector2 v2) => new((v1.X > v2.X) ? v1.X : v2.X, (v1.Y > v2.Y) ? v1.Y : v2.Y);
    public static void Distance(ref Vector2 v1, ref Vector2 v2, out double result)
    {
        double dx = v1.X - v2.X;
        double dy = v1.Y - v2.Y;
        result = Math.Sqrt(dx * dx + dy * dy);
    }
    public static void DistanceSquared(ref Vector2 v1, ref Vector2 v2, out double result)
    {
        double dx = v1.X - v2.X;
        double dy = v1.Y - v2.Y;
        result = (dx * dx) + (dy * dy);
    }
    public static void Add(ref Vector2 left, ref Vector2 right, out Vector2 result)
    {
        result.X = left.X + right.X;
        result.Y = left.Y + right.Y;
    }
    public static void Subtract(ref Vector2 left, ref Vector2 right, out Vector2 result)
    {
        result.X = left.X - right.X;
        result.Y = left.Y - right.Y;
    }
    public static void Multiply(ref Vector2 left, ref Vector2 right, out Vector2 result)
    {
        result.X = left.X * right.X;
        result.Y = left.Y * right.Y;
    }
    public static void Multiply(ref Vector2 left, double right, out Vector2 result)
    {
        result.X = left.X * right;
        result.Y = left.Y * right;
    }
    public static void Divide(ref Vector2 left, double right, out Vector2 result)
    {
        double invRight = 1 / right;
        result.X = left.X * invRight;
        result.Y = left.Y * invRight;
    }
    #endregion Fast ref methods
}
