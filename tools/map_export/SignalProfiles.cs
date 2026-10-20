// SPDX-License-Identifier: GPL-2.0-only
using System.Text.Json;
using TruckLib.Sii;

namespace Etsads.MapExport;
internal static class SignalProfiles
{
    public static async Task<int> Inspect(string[] args)
    {
        var options=Options.Parse(["inspect",..args.Skip(1)]);
        if(Directory.Exists(options.Output)||File.Exists(options.Output)) throw new IOException("Profile inspection output must be a new directory.");
        using var archives=ArchiveSet.Open(options); var fs=archives.FileSystem;
        var paths=fs.GetFiles("/def/world").Concat(fs.GetFiles("/def"))
            .Where(p=>Path.GetFileName(p).Contains("semaphore",StringComparison.OrdinalIgnoreCase)
                ||Path.GetFileName(p).Contains("traffic_light",StringComparison.OrdinalIgnoreCase)).Order(StringComparer.Ordinal).ToArray();
        paths = paths.Concat(fs.GetFiles("/def").Where(p=>Path.GetFileName(p).StartsWith("country",StringComparison.Ordinal)&&p.EndsWith(".sii",StringComparison.Ordinal))).Distinct().Order(StringComparer.Ordinal).ToArray();
        var sources=new Dictionary<string,object>(StringComparer.Ordinal);var units=new List<object>();
        foreach(var path in paths)
        {
            var sii=SiiFile.Open(path,fs);
            foreach(var asset in sii.Includes.Append(path).Distinct()) sources[asset]=archives.Origin(asset);
            units.AddRange(sii.Units.Select(u=>new { root_path=path,unit=Definitions.Inspect(u) }));
        }
        Directory.CreateDirectory(options.Output);
        await File.WriteAllTextAsync(Path.Combine(options.Output,"profiles.json"),JsonSerializer.Serialize(new {
            game_version=archives.Version, archive_inventory=archives.Inventory, paths,sources,units,
            scope="read-only actual SII definitions; no geometry or live activation acceptance" },Program.Json));
        Console.WriteLine(JsonSerializer.Serialize(new { output=options.Output,paths,units=units.Count,source_files=sources.Count },Program.Json));
        return 0;
    }
}
