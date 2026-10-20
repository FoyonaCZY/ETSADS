// SPDX-License-Identifier: GPL-2.0-only
using System.Numerics;
using TruckLib;
using TruckLib.Models.Ppd;
using TruckLib.ScsMap;

namespace Etsads.MapExport;

internal static class Geometry
{
    public static Vector3 Enu(Vector3 scs) => new(scs.X, -scs.Z, scs.Y);
    public static double Yaw(Quaternion rotation)
    {
        var forward = Enu(Vector3.Transform(-Vector3.UnitZ, rotation));
        return Math.Atan2(forward.Y, forward.X);
    }

    /// <summary>Resample by arc distance, preserving both endpoints and bounding sample separation.</summary>
    public static List<Vector3> Sample(Func<float, Vector3> curve, float estimatedLength, float spacing = 1)
    {
        if (!(spacing > 0) || !float.IsFinite(estimatedLength) || estimatedLength < 0)
            throw new ArgumentOutOfRangeException(nameof(spacing));
        var subdivisions = Math.Clamp((int)MathF.Ceiling(estimatedLength * 4), 16, 40000);
        var output = new List<Vector3> { curve(0) };
        var previous = output[0];
        var remaining = spacing;
        for (var i = 1; i <= subdivisions; i++)
        {
            var end = curve((float)i / subdivisions);
            var length = Vector3.Distance(previous, end);
            while (length >= remaining && length > 1e-6f)
            {
                previous = Vector3.Lerp(previous, end, remaining / length);
                output.Add(previous);
                length = Vector3.Distance(previous, end);
                remaining = spacing;
            }
            remaining -= length;
            previous = end;
        }
        var last = curve(1);
        if (Vector3.DistanceSquared(last, output[^1]) > 1e-8f) output.Add(last);
        if (output.Count == 1) output.Add(last);
        if (output.Any(p => !float.IsFinite(p.X) || !float.IsFinite(p.Y) || !float.IsFinite(p.Z)))
            throw new InvalidDataException("Curve contains non-finite coordinates.");
        return output;
    }

    public static List<Vector3> Road(Road road, float offset)
    {
        var length = Vector3.Distance(road.Node.Position, road.ForwardNode.Position);
        return Sample(t =>
        {
            var point = HermiteSpline.InterpolatePolyline(road.Node, road.ForwardNode, t);
            var tangent = HermiteSpline.DerivativePolyline(road.Node, road.ForwardNode, t);
            var normal = new Vector3(-tangent.Z, 0, tangent.X);
            if (normal.LengthSquared() > 1e-8f) point += Vector3.Normalize(normal) * offset;
            return Enu(point);
        }, Math.Max(length, road.Length));
    }

    public static List<Vector3> PrefabCurve(NavCurve curve, PrefabTransform transform)
    {
        return Sample(t => PrefabPoint(curve, transform.Apply, t), curve.Length);
    }

    public static Vector3 PrefabPoint(NavCurve curve, Func<Vector3, Vector3> transform, float t)
    {
        var start = Vector3.Transform(-Vector3.UnitZ * curve.Length, curve.StartRotation);
        var end = Vector3.Transform(-Vector3.UnitZ * curve.Length, curve.EndRotation);
        return Enu(transform(HermiteSpline.Interpolate(curve.StartPosition, curve.EndPosition, start, end, t)));
    }

    /// <summary>Travel direction at a lane end, over about the first/last metre:
    /// a road can begin with a sub-centimetre segment pointing anywhere, which
    /// made the link test reject a straight continuation (Sweden, r11).</summary>
    public static Vector3 Direction(IReadOnlyList<Vector3> points, bool start)
    {
        var end = start ? points[0] : points[^1];
        for (var i = 1; i < points.Count; i++)
        {
            var other = start ? points[i] : points[^(i + 1)];
            var delta = start ? other - end : end - other;
            if (delta.LengthSquared() >= 1.0f || i == points.Count - 1)
                return delta.LengthSquared() > 1e-8 ? Vector3.Normalize(delta) : Vector3.Zero;
        }
        return Vector3.Zero;
    }
}

/// <summary>Fit the actual placed control nodes; reject incorrect origin/rotation conventions.</summary>
internal sealed class PrefabTransform
{
    private readonly Vector3 localOrigin;
    private readonly Vector3 worldOrigin;
    private readonly float cosine;
    private readonly float sine;
    private readonly float heightSign;
    private readonly Quaternion? rigidRotation;
    public double MaxResidual { get; }
    public PrefabTransform(Prefab item, PrefabDescriptor descriptor)
    {
        if (item.Nodes.Count == 0 || item.Origin >= descriptor.Nodes.Count)
            throw new InvalidDataException("Prefab origin is outside its control-node list.");
        localOrigin = descriptor.Nodes[item.Origin].Position;
        worldOrigin = item.Nodes[0].Position;
        var a = 0d;
        var b = 0d;
        for (var i = 1; i < item.Nodes.Count && i < descriptor.Nodes.Count; i++)
        {
            var local = descriptor.Nodes[(i + item.Origin) % descriptor.Nodes.Count].Position - localOrigin;
            var world = item.Nodes[i].Position - worldOrigin;
            a += local.X * world.X + local.Z * world.Z;
            b += local.X * world.Z - local.Z * world.X;
        }
        double angle;
        if (a * a + b * b > 1e-6) angle = Math.Atan2(b, a);
        else
        {
            var localDirection = descriptor.Nodes[item.Origin].Direction;
            var worldDirection = Vector3.Transform(-Vector3.UnitZ, item.Nodes[0].Rotation);
            angle = Math.Atan2(worldDirection.Z, worldDirection.X) - Math.Atan2(localDirection.Z, localDirection.X);
        }
        cosine = (float)Math.Cos(angle);
        sine = (float)Math.Sin(angle);
        // PPD vertical position is conventionally positive up. Test both against actual map nodes.
        double positive = 0, negative = 0;
        for (var i = 0; i < item.Nodes.Count && i < descriptor.Nodes.Count; i++)
        {
            var local = descriptor.Nodes[(i + item.Origin) % descriptor.Nodes.Count].Position.Y - localOrigin.Y;
            var world = item.Nodes[i].Position.Y - worldOrigin.Y;
            positive += Math.Pow(local - world, 2);
            negative += Math.Pow(-local - world, 2);
        }
        heightSign = negative + 1e-5 < positive ? -1 : 1;
        var pairs = Enumerable.Range(0, Math.Min(item.Nodes.Count, descriptor.Nodes.Count))
            .Select(i => (Local: descriptor.Nodes[(i + item.Origin) % descriptor.Nodes.Count].Position - localOrigin,
                World: item.Nodes[i].Position - worldOrigin)).ToArray();
        if (pairs.Count(p => p.Local.LengthSquared() > 1) >= 1)
        {
            var fitPairs = pairs.Select(p => (Local: new Vector3(p.Local.X, p.Local.Y * heightSign, p.Local.Z), p.World)).ToList();
            var longest = fitPairs.MaxBy(p => p.Local.LengthSquared());
            if (fitPairs.All(p => Vector3.Cross(p.Local, longest.Local).LengthSquared() < longest.Local.LengthSquared() * 0.0001f))
            {
                // Collinear controls leave roll underdetermined. Preserve the up directions projected
                // perpendicular to the baseline, instead of accepting an arbitrary eigenvector roll.
                var l = Vector3.Normalize(longest.Local); var w = Vector3.Normalize(longest.World);
                var localUp = Vector3.UnitY - l * Vector3.Dot(Vector3.UnitY, l);
                var worldUp = Vector3.UnitY - w * Vector3.Dot(Vector3.UnitY, w);
                if (localUp.LengthSquared() > 1e-4 && worldUp.LengthSquared() > 1e-4)
                    fitPairs.Add((Vector3.Normalize(localUp), Vector3.Normalize(worldUp)));
            }
            var candidate = FitRotation(fitPairs);
            var rigidError = pairs.Sum(p => Vector3.DistanceSquared(Vector3.Transform(new Vector3(p.Local.X, p.Local.Y * heightSign, p.Local.Z), candidate), p.World));
            var yawError = pairs.Sum(p => Vector3.DistanceSquared(Apply(localOrigin + p.Local), worldOrigin + p.World));
            if (rigidError + 1e-7 < yawError) rigidRotation = candidate;
        }
        foreach (var pair in pairs) MaxResidual = Math.Max(MaxResidual, Vector3.Distance(Apply(localOrigin + pair.Local), worldOrigin + pair.World));
        if (MaxResidual > 0.5)
        {
            var invariantError = 0d;
            foreach (var p in pairs) foreach (var q in pairs)
                invariantError = Math.Max(invariantError, Math.Abs(Vector3.Distance(p.Local, q.Local) - Vector3.Distance(p.World, q.World)));
            throw new InvalidDataException($"Prefab {item.Uid} placement residual {MaxResidual:F3} m exceeds 0.5 m; maximum pairwise-distance mismatch {invariantError:F3} m (rigid rotation preserves distances).");
        }
    }
    public Vector3 Apply(Vector3 position)
    {
        var p = position - localOrigin;
        if (rigidRotation is { } rotation) return worldOrigin + Vector3.Transform(new Vector3(p.X, p.Y * heightSign, p.Z), rotation);
        return worldOrigin + new Vector3(cosine * p.X - sine * p.Z, heightSign * p.Y, sine * p.X + cosine * p.Z);
    }

    /// <summary>Horn's rigid rotation solution; Jacobi diagonalisation avoids iterative eigenvalue sign ambiguity.</summary>
    internal static Quaternion FitRotation(IEnumerable<(Vector3 Local, Vector3 World)> pairs)
    {
        var s = new double[3, 3];
        foreach (var (local, world) in pairs)
        {
            var a = new[] { local.X, local.Y, local.Z };
            var b = new[] { world.X, world.Y, world.Z };
            for (var i = 0; i < 3; i++) for (var j = 0; j < 3; j++) s[i, j] += a[i] * b[j];
        }
        var n = new double[4, 4];
        n[0, 0] = s[0, 0] + s[1, 1] + s[2, 2];
        n[1, 1] = s[0, 0] - s[1, 1] - s[2, 2];
        n[2, 2] = -s[0, 0] + s[1, 1] - s[2, 2];
        n[3, 3] = -s[0, 0] - s[1, 1] + s[2, 2];
        n[0, 1] = n[1, 0] = s[1, 2] - s[2, 1];
        n[0, 2] = n[2, 0] = s[2, 0] - s[0, 2];
        n[0, 3] = n[3, 0] = s[0, 1] - s[1, 0];
        n[1, 2] = n[2, 1] = s[0, 1] + s[1, 0];
        n[1, 3] = n[3, 1] = s[0, 2] + s[2, 0];
        n[2, 3] = n[3, 2] = s[1, 2] + s[2, 1];
        var vectors = new double[4, 4];
        for (var i = 0; i < 4; i++) vectors[i, i] = 1;
        for (var iteration = 0; iteration < 80; iteration++)
        {
            var p = 0; var q = 1;
            for (var i = 0; i < 4; i++) for (var j = i + 1; j < 4; j++)
                if (Math.Abs(n[i, j]) > Math.Abs(n[p, q])) { p = i; q = j; }
            if (Math.Abs(n[p, q]) < 1e-10) break;
            var angle = 0.5 * Math.Atan2(2 * n[p, q], n[q, q] - n[p, p]);
            var c = Math.Cos(angle); var t = Math.Sin(angle);
            var pp = n[p, p]; var qq = n[q, q]; var pq = n[p, q];
            for (var i = 0; i < 4; i++)
            {
                if (i != p && i != q)
                {
                    var ip = n[i, p]; var iq = n[i, q];
                    n[i, p] = n[p, i] = c * ip - t * iq;
                    n[i, q] = n[q, i] = t * ip + c * iq;
                }
                var vp = vectors[i, p]; var vq = vectors[i, q];
                vectors[i, p] = c * vp - t * vq;
                vectors[i, q] = t * vp + c * vq;
            }
            n[p, p] = c * c * pp - 2 * c * t * pq + t * t * qq;
            n[q, q] = t * t * pp + 2 * c * t * pq + c * c * qq;
            n[p, q] = n[q, p] = 0;
        }
        var best = Enumerable.Range(0, 4).MaxBy(i => n[i, i]);
        return Quaternion.Normalize(new Quaternion((float)vectors[1, best], (float)vectors[2, best], (float)vectors[3, best], (float)vectors[0, best]));
    }
    public double Yaw(Quaternion rotation)
    {
        var local = Vector3.Transform(-Vector3.UnitZ, rotation);
        var world = Geometry.Enu(Apply(localOrigin + local) - worldOrigin);
        return Math.Atan2(world.Y, world.X);
    }
}

internal static class GeometryTests
{
    public static async Task<int> Run()
    {
        static void Require(bool condition, string name) { if (!condition) throw new InvalidOperationException("Self-test failed: " + name); }
        Require(Geometry.Enu(new(1, 2, 3)) == new Vector3(1, -3, 2), "SCS to ENU");
        var points = Geometry.Sample(t => new Vector3(100 * t, 0, 0), 100);
        Require(points.Count == 101 && points[0].X == 0 && points[^1].X == 100, "endpoint preserving arc sampling");
        var curve = Geometry.Sample(t => new Vector3(10 * MathF.Cos(t * MathF.PI), 10 * MathF.Sin(t * MathF.PI), 0), 32);
        Require(curve.Zip(curve.Skip(1)).All(p => Vector3.Distance(p.First, p.Second) <= 1.001), "bounded curved sample separation");
        Require(Math.Abs(Geometry.Yaw(Quaternion.Identity) - Math.PI / 2) < 1e-6, "north heading");
        var expectedRotation = Quaternion.CreateFromYawPitchRoll(1.2f, 0.25f, -0.1f);
        var basis = new[] { Vector3.UnitX * 50, Vector3.UnitZ * 80, Vector3.UnitY * 20 };
        var actualRotation = PrefabTransform.FitRotation(basis.Select(p => (p, Vector3.Transform(p, expectedRotation))));
        Require(basis.All(p => Vector3.Distance(Vector3.Transform(p, actualRotation), Vector3.Transform(p, expectedRotation)) < 1e-4), "3D prefab rigid rotation fit");
        ConflictTests.Run();
        var path = Path.Combine(Path.GetTempPath(), "etsads-map-test-" + Guid.NewGuid().ToString("N") + ".parquet");
        try
        {
            await Parquet.Serialization.ParquetSerializer.SerializeAsync(new[] { new NodeRow(ulong.MaxValue, 1, 2, 3) }, path);
            var roundtrip = await Parquet.Serialization.ParquetSerializer.DeserializeAsync<NodeRow>(path);
            Require(roundtrip.Data.Count == 1 && roundtrip.Data[0].uid == ulong.MaxValue, "Parquet native u64 lossless round trip");
        }
        finally { File.Delete(path); }
        Console.WriteLine("Passed: coordinates, arc sampling, heading, 3D placement, native/derived conflicts, overpass/adjacency exclusion, priority rules, Parquet UInt64 round trip.");
        return 0;
    }
}
