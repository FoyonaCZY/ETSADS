// SPDX-License-Identifier: GPL-2.0-only
using TruckLib.Models.Ppd;
using TruckLib.ScsMap;

namespace Etsads.MapExport;

// Read-only: say which items a native GPS node belongs to and why the export
// left it out, so a route the game plans through an unexported node can be
// traced to the item type or definition the exporter does not handle.
internal static class NodeProbe
{
    public static void Run(Map map, Definitions definitions, TruckLib.IFileSystem fs, IReadOnlyList<ulong> uids)
    {
        foreach (var uid in uids)
        {
            if (!map.Nodes.TryGetValue(uid, out var node)) { Console.WriteLine($"node {uid}: not in the loaded sectors"); continue; }
            var p = Geometry.Enu(node.Position);
            Console.WriteLine($"node {uid}: enu {p.X:F1} {p.Y:F1} {p.Z:F1}");
            Describe("  forward", node.ForwardItem, uid, definitions, fs);
            Describe("  backward", node.BackwardItem, uid, definitions, fs);
        }
        var roads = map.MapItems.Values.OfType<Road>().ToArray();
        var laneless = roads.Where(r => !definitions.RoadLooks.TryGetValue(r.RoadType.String, out var look)
            || definitions.ResolveLanes(look, "left", r.Left).Length + definitions.ResolveLanes(look, "right", r.Right).Length == 0).ToArray();
        Console.WriteLine($"roads {roads.Length}, without lanes {laneless.Length}");
        foreach (var group in laneless.GroupBy(r => $"{r.RoadType.String} L={r.Left.Variant.String} R={r.Right.Variant.String}").OrderByDescending(g => g.Count()).Take(25))
            Console.WriteLine($"  {group.Count(),5} {group.Key}");
    }

    private static void Describe(string label, IMapObject? item, ulong uid, Definitions definitions, TruckLib.IFileSystem fs)
    {
        switch (item)
        {
            case null: Console.WriteLine($"{label}: none"); break;
            case Road road:
                var hasLook = definitions.RoadLooks.TryGetValue(road.RoadType.String, out var look);
                var left = hasLook ? definitions.ResolveLanes(look!, "left", road.Left).Length : -1;
                var right = hasLook ? definitions.ResolveLanes(look!, "right", road.Right).Length : -1;
                Console.WriteLine($"{label}: road {road.Uid} look {road.RoadType.String} found={hasLook} lanes L={left} R={right} variants L={road.Left.Variant.String} R={road.Right.Variant.String}");
                break;
            case Prefab prefab:
                var hasModel = definitions.Prefabs.TryGetValue(prefab.Model.String, out var path);
                var text = $"{label}: prefab {prefab.Uid} model {prefab.Model.String} found={hasModel} nodes={prefab.Nodes.Count} origin={prefab.Origin}";
                if (hasModel)
                {
                    try
                    {
                        var descriptor = PrefabDescriptor.Open(path!, fs);
                        var index = prefab.Nodes.ToList().FindIndex(n => n.Uid == uid);
                        var d = index < 0 ? -1 : (index + prefab.Origin) % prefab.Nodes.Count;
                        text += $" desc={path} desc_nodes={descriptor.Nodes.Count} curves={descriptor.NavCurves.Count} slot={index}->{d}";
                        if (d >= 0 && d < descriptor.Nodes.Count)
                            text += $" in=[{string.Join(',', descriptor.Nodes[d].InputLines)}] out=[{string.Join(',', descriptor.Nodes[d].OutputLines)}]";
                    }
                    catch (Exception ex) { text += $" descriptor failed: {ex.Message}"; }
                }
                Console.WriteLine(text);
                break;
            default: Console.WriteLine($"{label}: {item.GetType().Name} {item.Uid}"); break;
        }
    }
}
