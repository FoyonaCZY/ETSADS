// SPDX-License-Identifier: GPL-2.0-only
using System.Text.Json;
using TruckLib.ScsMap;

namespace Etsads.MapExport;

public sealed record FerryRow(ulong uid, string port, bool train, ulong prefab, double x, double y, double z, double yaw)
{
    public FerryRow() : this(0, "", false, 0, 0, 0, 0, 0) { }
}

// Ferry and train-transport activation points for the runtime: where a
// truck following a route across the water stops and asks for the crossing.
internal static class Ferries
{
    public const string File = "ferries.json";

    public static FerryRow Row(Ferry ferry)
    {
        var p = Geometry.Enu(ferry.Node.Position);
        return new(ferry.Uid, ferry.Port.String, ferry.TrainTransport, ferry.Prefab?.Uid ?? 0, p.X, p.Y, p.Z, Geometry.Yaw(ferry.Node.Rotation));
    }

    /// Activation points of a loaded map (`ferries` command, for a map
    /// generated before the export carried them).
    public static IEnumerable<FerryRow> Collect(Map map) =>
        map.MapItems.Values.OfType<Ferry>().Select(Row);

    public static async Task Write(IEnumerable<FerryRow> rows, string path)
    {
        var ferries = rows.ToArray();
        await System.IO.File.WriteAllBytesAsync(path, JsonSerializer.SerializeToUtf8Bytes(new { schema_version = 1, ferries }));
        Console.WriteLine($"Wrote {ferries.Length} ferry points to {path}.");
    }
}
