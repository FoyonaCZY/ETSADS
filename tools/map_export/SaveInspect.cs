// SPDX-License-Identifier: GPL-2.0-only
using System.Security.Cryptography;
using System.Text;
using System.Text.Json;
using TruckLib.Sii;

namespace Etsads.MapExport;

// Read-only inspection for choosing an existing game checkpoint. The tool
// never rewrites a save, changes save format or claims unsupported binary data
// was decoded into a known vehicle state.
internal static class SaveInspect
{
    public static int Run(string[] args)
    {
        if (args.Length != 3 || args[1] != "--input")
            throw new ArgumentException("Expected inspect-save --input info.sii (or game.sii).");
        var path = Path.GetFullPath(args[2]);
        var info = new FileInfo(path);
        if (info.Length > 16 * 1024 * 1024) throw new InvalidDataException("Save input exceeds 16 MiB budget.");
        var bytes = File.ReadAllBytes(path);
        var decoded = SiiFile.Decode(bytes);
        if (decoded.Length > 64 * 1024 * 1024) throw new InvalidDataException("Decoded save exceeds 64 MiB budget.");
        var magic = Encoding.ASCII.GetString(decoded.AsSpan(0, Math.Min(4, decoded.Length)));
        object? units = null;
        if (magic == "SiiN")
        {
            var parsed = SiiFile.Load(decoded, info.DirectoryName!);
            var selected = parsed.Units.Where(unit => unit.Class is "save_container" or "player" or "vehicle").Take(16);
            units = selected.Select(unit => new { unit.Class, unit.Name, attributes = unit.Attributes
                .Where(pair => unit.Class == "save_container" || pair.Key is "truck_placement" or "trailer_placement"
                    or "assigned_truck" or "my_truck" or "accessories" or "engine" or "chassis")
                .ToDictionary(pair => pair.Key, pair => Definitions.Text(pair.Value)) }).ToArray();
        }
        Console.WriteLine(JsonSerializer.Serialize(new { schema_version = 1, source = "local_game_save", path,
            bytes = bytes.Length, sha256 = Convert.ToHexStringLower(SHA256.HashData(bytes)),
            decoded_bytes = decoded.Length, decoded_format = magic, text_supported = magic == "SiiN", units }, Program.Json));
        return 0;
    }
}
