// SPDX-License-Identifier: GPL-2.0-only
using System.Numerics;
using TruckLib.Models.Ppd;

namespace Etsads.MapExport;

public sealed record ConflictPairRow(string id, ulong owner_uid, string lane_a, string lane_b, string kind,
    string source, double x, double y, double z, float radius_m, double geometry_gap_m,
    int? priority_a, int? priority_b, string relation, bool conservative_wait)
{
    public ConflictPairRow() : this("", 0, "", "", "", "", 0, 0, 0, 0, 0, null, null, "unknown", true) { }
}

internal static class ConflictTests
{
    public static void Run()
    {
        static void Check(bool condition, string message) { if (!condition) throw new InvalidOperationException("Conflict test failed: " + message); }
        var a = new NavCurve { StartPosition = new(-5, 0, 0), EndPosition = new(5, 0, 0), Length = 10,
            StartRotation = Quaternion.CreateFromYawPitchRoll(-MathF.PI / 2, 0, 0),
            EndRotation = Quaternion.CreateFromYawPitchRoll(-MathF.PI / 2, 0, 0), SemaphoreId = -1 };
        var b = new NavCurve { StartPosition = new(0, 0, -5), EndPosition = new(0, 0, 5), Length = 10,
            StartRotation = Quaternion.CreateFromYawPitchRoll(MathF.PI, 0, 0),
            EndRotation = Quaternion.CreateFromYawPitchRoll(MathF.PI, 0, 0), SemaphoreId = -1 };
        var ppd = new PrefabDescriptor { NavCurves = [a, b], Intersections =
        [
            new Intersection { CurveId = 0, Position = 5, Radius = 4, Flags = new TruckLib.FlagField(0x20) },
            new Intersection { CurveId = 1, Position = 5, Radius = 4, Flags = new TruckLib.FlagField(0) },
        ] };
        var definitions = new Definitions();
        ConflictResult Run() => ConflictBuilder.Build(7, ppd, ppd.NavCurves.Select((curve, index) => (curve, index))
            .ToDictionary(pair => pair.index, pair => Geometry.Sample(t => Geometry.PrefabPoint(pair.curve, p => p, t), pair.curve.Length)),
            (curve, t) => Geometry.PrefabPoint(curve, p => p, t), definitions);
        var result = Run();
        Check(result.Pairs.Count == 1 && result.Pairs[0].source == "native_ppd_group"
            && Math.Abs(result.Pairs[0].x) < 1e-5 && result.Pairs[0].relation == "unknown" && result.Pairs[0].conservative_wait,
            "native sibling grouping, distance/length parameter and unknown priority");
        ppd.Intersections.Clear();
        Check(Run().Pairs is [{ source: "derived_geometry" }], "derived crossing fallback");
        b.StartPosition += Vector3.UnitY * 3; b.EndPosition += Vector3.UnitY * 3;
        Check(Run().Pairs.Count == 0, "overpass is not a conflict");
        b.StartPosition -= Vector3.UnitY * 3; b.EndPosition -= Vector3.UnitY * 3;
        a.NextLines.Add(1);
        Check(Run().Pairs.Count == 0, "topological neighbors excluded");
        a.NextLines.Clear();
        a.TrafficRule = "major_road"; b.TrafficRule = "minor_road";
        definitions.TrafficRules["traffic_rule.major_road"] = new TruckLib.Sii.Unit("traffic_rule_data", "traffic_rule.major_road")
            { Attributes = new() { ["rule"] = "priority", ["num_params"] = new List<float> { -1, 0, 2 } } };
        definitions.TrafficRules["traffic_rule.minor_road"] = new TruckLib.Sii.Unit("traffic_rule_data", "traffic_rule.minor_road")
            { Attributes = new() { ["rule"] = "priority", ["num_params"] = new List<float> { -1, 0, 1 } } };
        Check(Run().Pairs[0].relation == "b_yields_a", "official major/minor priority semantics");
        a.SemaphoreId = 0;
        Check(Run().Pairs[0].relation == "signal_state_required", "live signal takes precedence over static priority");
    }
}
public sealed record LaneControlRow(string lane_id, string traffic_rule, string control_kind,
    int? priority_modifier, bool additive_priority, string? signal_group_id, string evidence_source)
{
    public LaneControlRow() : this("", "", "unknown", null, false, null, "") { }
}
internal sealed record CurveControl(string Kind, int? Priority, bool Additive, string? SignalGroup);
internal sealed record ConflictResult(List<ConflictPairRow> Pairs, List<LaneControlRow> Controls, int InvalidNativeGroups);

/// <summary>
/// Native group flags follow SCS's published PPD layout and BlenderTools exporter.
/// Position is serialized as (Hermite parameter * curve length), not a world position.
/// https://modding.scssoft.com/wiki/Games/ETS2/Modding_guides/1.9
/// https://github.com/SCSSoftware/BlenderTools/blob/master/addon/io_scs_tools/exp/pip/exporter.py
/// </summary>
internal static class ConflictBuilder
{
    internal static ConflictResult Build(ulong owner, PrefabDescriptor ppd,
        IReadOnlyDictionary<int, List<Vector3>> samples, Func<NavCurve, float, Vector3> pointAt, Definitions definitions)
    {
        var pairs = new List<ConflictPairRow>();
        var controls = ppd.NavCurves.Select((curve, index) => Control(curve, owner, definitions)).ToArray();
        var controlRows = controls.Select((control, index) => new LaneControlRow(Id(index), ppd.NavCurves[index].TrafficRule.String,
            control.Kind, control.Priority, control.Additive, control.SignalGroup, "native_ppd_and_sii_traffic_rule")).ToList();
        var invalid = 0;
        var represented = new HashSet<(int A, int B)>();
        for (var offset = 0; offset < ppd.Intersections.Count;)
        {
            var head = ppd.Intersections[offset];
            var count = (int)((head.Flags.Bits & 0xf0) >> 4);
            if (count < 2 || offset + count > ppd.Intersections.Count) { invalid++; offset++; continue; }
            var group = ppd.Intersections.Skip(offset).Take(count).ToArray();
            var valid = group.Select((record, index) => record.CurveId < ppd.NavCurves.Count
                && (index == 0 || (record.Flags.Bits & 0xf0) == 0)
                && float.IsFinite(record.Position) && record.Position >= -0.001f
                && record.Position <= ppd.NavCurves[(int)record.CurveId].Length + 0.001f).All(value => value);
            if (!valid) { invalid++; offset += count; continue; }
            var kind = (head.Flags.Bits & 0x10000) != 0 ? "fork"
                : (head.Flags.Bits & 0x20000) != 0 ? "merge"
                : (head.Flags.Bits & 0x40000) != 0 ? "cross_sharp" : "cross";
            for (var a = 0; a < count; a++) for (var b = a + 1; b < count; b++)
            {
                var ca = (int)group[a].CurveId; var cb = (int)group[b].CurveId;
                if (ca == cb || kind == "fork" || Adjacent(ppd, ca, cb)) continue;
                var ac = ppd.NavCurves[ca]; var bc = ppd.NavCurves[cb];
                if (ac.Length <= 1e-5 || bc.Length <= 1e-5) { invalid++; continue; }
                var pa = pointAt(ac, Math.Clamp(group[a].Position / ac.Length, 0, 1));
                var pb = pointAt(bc, Math.Clamp(group[b].Position / bc.Length, 0, 1));
                var gap = Vector3.Distance(pa, pb);
                if (gap > 0.5) { invalid++; continue; }
                represented.Add(Key(ca, cb));
                Add(ca, cb, kind, "native_ppd_group", (pa + pb) / 2, Math.Max(group[a].Radius, group[b].Radius), gap);
            }
            offset += count;
        }
        // Geometry fallback is needed when native records are missing or fail consistency checks.
        // It never assigns a right of way without independent rule evidence.
        if (ppd.Intersections.Count == 0 || invalid > 0)
        {
            foreach (var a in samples.Keys) foreach (var b in samples.Keys.Where(b => b > a))
            {
                if (represented.Contains(Key(a, b)) || Adjacent(ppd, a, b)) continue;
                var ap = samples[a]; var bp = samples[b];
                if (Vector3.Distance(ap[0], bp[0]) < 0.05) continue; // Common fork is not a crossing conflict.
                if (!BoundsOverlap(ap, bp)) continue;
                var intersections = new List<Vector3>();
                if (Vector3.Distance(ap[^1], bp[^1]) < 0.1)
                {
                    var p = (ap[^1] + bp[^1]) / 2;
                    intersections.Add(p);
                    Add(a, b, "merge", "derived_geometry", p, 4, Vector3.Distance(ap[^1], bp[^1]));
                }
                for (var ai = 1; ai < ap.Count; ai++) for (var bi = 1; bi < bp.Count; bi++)
                    if (SegmentIntersection(ap[ai - 1], ap[ai], bp[bi - 1], bp[bi], out var p, out var gap))
                    {
                        if (intersections.Any(previous => Vector3.Distance(previous, p) < 0.25)) continue;
                        var kind = Vector3.Distance(ap[^1], bp[^1]) < 0.1 ? "merge" : "cross";
                        Add(a, b, kind, "derived_geometry", p, 4, gap);
                        intersections.Add(p);
                    }
            }
        }
        return new(pairs, controlRows, invalid);

        string Id(int curve) => $"{owner}:prefab:{curve}";
        void Add(int a, int b, string kind, string source, Vector3 p, float radius, double gap)
        {
            var relation = Relation(controls[a], controls[b], kind);
            pairs.Add(new($"{owner}:conflict:{pairs.Count}", owner, Id(a), Id(b), kind, source,
                p.X, p.Y, p.Z, radius, gap, controls[a].Priority, controls[b].Priority,
                relation, relation is "unknown" or "signal_state_required" or "equal_priority" or "zip_merge"));
        }
    }

    private static (int, int) Key(int a, int b) => (Math.Min(a, b), Math.Max(a, b));
    private static bool Adjacent(PrefabDescriptor ppd, int a, int b) =>
        ppd.NavCurves[a].NextLines.Contains(b) || ppd.NavCurves[a].PreviousLines.Contains(b)
        || ppd.NavCurves[b].NextLines.Contains(a) || ppd.NavCurves[b].PreviousLines.Contains(a);

    private static CurveControl Control(NavCurve curve, ulong owner, Definitions definitions)
    {
        var token = curve.TrafficRule.String;
        var pending = new Queue<string>();
        if (!string.IsNullOrEmpty(token)) pending.Enqueue(token.StartsWith("traffic_rule.", StringComparison.Ordinal) ? token : "traffic_rule." + token);
        var visited = new HashSet<string>();
        var kind = "uncontrolled";
        int? priority = (int)curve.PriorityModifier > 0 ? (int)curve.PriorityModifier : null;
        var additive = curve.AdditivePriority;
        var incomplete = false;
        while (pending.TryDequeue(out var name))
        {
            if (!visited.Add(name)) continue;
            if (!definitions.TrafficRules.TryGetValue(name, out var rule)) { incomplete = true; continue; }
            foreach (var nested in Definitions.Array(rule, "additional_rules").Select(Definitions.Text)) pending.Enqueue(nested);
            var command = rule.Attributes.TryGetValue("rule", out object? value) ? Definitions.Text(value) : "";
            var numbers = Definitions.Array(rule, "num_params").Select(Convert.ToSingle).ToArray();
            if (command == "priority" && numbers.Length >= 3)
            {
                if (numbers[2] == 1 && kind != "stop") kind = "yield";
                if (numbers[2] == 2 && kind is not ("yield" or "stop")) kind = "priority_road";
                if (numbers[0] >= 0) { priority = (int)numbers[0]; additive = numbers[1] != 0; }
            }
            // Other 'stop' commands include speed bumps and train dwell: do not mislabel them as stop signs.
            if (command == "stop" && name == "traffic_rule.stop") kind = "stop";
        }
        if (incomplete) kind = "unknown";
        var signal = curve.SemaphoreId >= 0 ? $"{owner}:signal:{curve.SemaphoreId}" : null;
        if (signal is not null) kind = "signal";
        return new(kind, priority, additive, signal);
    }

    private static string Relation(CurveControl a, CurveControl b, string kind)
    {
        if (a.SignalGroup is not null || b.SignalGroup is not null) return "signal_state_required";
        if (a.Kind == "unknown" || b.Kind == "unknown") return "unknown";
        if (a.Kind is "stop" or "yield" && b.Kind == "priority_road") return "a_yields_b";
        if (b.Kind is "stop" or "yield" && a.Kind == "priority_road") return "b_yields_a";
        if (a.Priority is { } ap && b.Priority is { } bp && !a.Additive && !b.Additive)
            return ap > bp ? "b_yields_a" : ap < bp ? "a_yields_b" : kind == "merge" ? "zip_merge" : "equal_priority";
        return "unknown";
    }

    private static bool BoundsOverlap(List<Vector3> a, List<Vector3> b) =>
        a.Min(p => p.X) <= b.Max(p => p.X) && a.Max(p => p.X) >= b.Min(p => p.X)
        && a.Min(p => p.Y) <= b.Max(p => p.Y) && a.Max(p => p.Y) >= b.Min(p => p.Y)
        && a.Min(p => p.Z) <= b.Max(p => p.Z) + 0.5 && a.Max(p => p.Z) >= b.Min(p => p.Z) - 0.5;

    internal static bool SegmentIntersection(Vector3 a, Vector3 b, Vector3 c, Vector3 d, out Vector3 point, out double verticalGap)
    {
        point = default; verticalGap = 0;
        var rx = (double)b.X - a.X; var ry = (double)b.Y - a.Y;
        var sx = (double)d.X - c.X; var sy = (double)d.Y - c.Y;
        var denominator = rx * sy - ry * sx;
        if (Math.Abs(denominator) < 1e-9) return false;
        var cx = (double)c.X - a.X; var cy = (double)c.Y - a.Y;
        var t = (cx * sy - cy * sx) / denominator;
        var u = (cx * ry - cy * rx) / denominator;
        if (t < -1e-6 || t > 1.000001 || u < -1e-6 || u > 1.000001) return false;
        var pa = Vector3.Lerp(a, b, (float)t); var pb = Vector3.Lerp(c, d, (float)u);
        verticalGap = Math.Abs(pa.Z - pb.Z);
        if (verticalGap > 0.5) return false;
        point = (pa + pb) / 2;
        return true;
    }
}
