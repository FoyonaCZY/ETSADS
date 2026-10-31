// SPDX-License-Identifier: GPL-2.0-only
using System.Text.Json;
using TruckLib.Models.Ppd;
using TruckLib.ScsMap;

namespace Etsads.MapExport;

// Fuel pumps for the runtime: every fuel-station spawn point of a prefab.
// The pump prefabs are placed over a service road whose lanes pass beside
// them; the runtime finds those lanes itself.
internal static class FuelStations
{
    public const string File = "fuel_stations.json";

    /// Pumps of the prefabs in a loaded map (`fuel-stations` command, for a
    /// map generated before the export carried them).
    public static IEnumerable<PoiRow> Collect(Map map, Definitions definitions, TruckLib.IFileSystem fs)
    {
        var cache = new Dictionary<string, PrefabDescriptor?>();
        foreach (var prefab in map.MapItems.Values.OfType<Prefab>())
        {
            if (!definitions.Prefabs.TryGetValue(prefab.Model.String, out var path)) continue;
            if (!cache.TryGetValue(path, out var descriptor))
            {
                try { descriptor = PrefabDescriptor.Open(path, fs); }
                catch (Exception ex) when (ex is InvalidDataException or FileNotFoundException or NotSupportedException) { descriptor = null; }
                cache[path] = descriptor;
            }
            if (descriptor is null || descriptor.Nodes.Count == 0) continue;
            if (!descriptor.SpawnPoints.Any(s => s.Type == SpawnPointType.GasStation)) continue;
            var transform = new PrefabTransform(prefab, descriptor);
            for (var i = 0; i < descriptor.SpawnPoints.Count; i++)
            {
                var spawn = descriptor.SpawnPoints[i];
                if (spawn.Type != SpawnPointType.GasStation) continue;
                var p = Geometry.Enu(transform.Apply(spawn.Position));
                yield return new($"{prefab.Uid}:spawn:{i}", prefab.Uid, spawn.Type.ToString(), p.X, p.Y, p.Z, transform.Yaw(spawn.Rotation));
            }
        }
    }

    public static async Task Write(IEnumerable<PoiRow> pois, string path)
    {
        var pumps = pois.Where(p => p.kind == nameof(SpawnPointType.GasStation))
            .Select(p => new { id = p.id, prefab = p.owner_uid, x = p.x, y = p.y, z = p.z, yaw = p.yaw })
            .ToArray();
        await System.IO.File.WriteAllBytesAsync(path, JsonSerializer.SerializeToUtf8Bytes(new { schema_version = 1, pumps }));
        Console.WriteLine($"Wrote {pumps.Length} fuel pumps to {path}.");
    }
}
