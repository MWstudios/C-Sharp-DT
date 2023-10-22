# C#DT - a C# port of CDT
This is a C# port of artem-ogre's [CDT library](https://github.com/artem-ogre/CDT). Make sure to read the original README, it contains more information.

There are a few notable problems, such as CDT crashing and asserting at thin triangles when `minDistToConstraintEdge` is set to too low. However, there also may be some typos in my code. If you have found any other discrepancies or strange behavior, report it in Issues. _Be aware that any bug you have found could have existed on the C++ side as well, so make sure to test both._

## Adding to your project

The code is fairly small. You can simply drag and drop all .cs files from the src folder in your project. These should be able to get compiled starting with .NET 5.

If you have your own implementation of Vector2 that is 64-bit and supports element-wise arithmetic, `Vector2.Distance()`, `Vector2.DistanceSquared()`, `Vector2.Min()` and `Vector2.Max()`, you do not need Vector.cs. But be sure to link your namespace under the other 3 files.

32-bit floats will _not_ work unless you replace all `double`s with `float` and all `Math.ScaleB(1, -52)` with `Math.ScaleB(1, -24)` in Triangle.cs, but even then it is _not_ recommended.

## Using CDT in your code
The syntax is similar to C++:
```csharp
CDT cdt = new(VertexInsertionOrder.Auto, //or AsProvided if forced order
    IntersectingConstraintEdges.Resolve, //or Ignore if no intersecting edges
    10); //unlikely to crash within this tolerance, but thin triangles phase through edges

cdt.InsertVertices(new Vector2[] { new(100, 200), new(300, 400), /* ... */ });
cdt.InsertEdges(new Edge[] { new(0, 1), new(2, 3), /* ... */ });
```
When `InsertVertices` is first called (and `superGeomType` is set to `SuperTriangle` by default), CDT makes a large triangle encompassing all vertices. The triangulation itself then occurs when outer unedged triangles, holes or the super triangle is erased:
```csharp
cdt.EraseSuperTriangle();
// or
cdt.EraseOuterEdges();
// or
cdt.EraseOuterEdgesAndHoles();
```
