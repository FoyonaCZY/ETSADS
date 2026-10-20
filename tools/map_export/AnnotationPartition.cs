// SPDX-License-Identifier: GPL-2.0-only
using System.IO.Compression;
using System.Security.Cryptography;
using System.Text.Json;
using Parquet.Serialization;

namespace Etsads.MapExport;

internal static class AnnotationPartition
{
    public static async Task<int> Run(string[] args)
    {
        if (args.Length < 5 || args.Length % 2 != 1 || args[1] != "--input" || args[3] != "--output")
            throw new ArgumentException("Expected partition-metadata --input ORIGINAL_PARQUET --output EXISTING_SECTOR_DIRECTORY.");
        var input = Path.GetFullPath(args[2]); var output = Path.GetFullPath(args[4]);
        string? game = null; var mods = new List<string>();
        for(var i=5;i<args.Length;i+=2) { if(args[i]=="--game-dir" && game is null)game=Path.GetFullPath(args[i+1]);else if(args[i]=="--mod")mods.Add(Path.GetFullPath(args[i+1]));else throw new ArgumentException("Unknown metadata argument."); }
        if (File.Exists(Path.Combine(output, "traffic_index.json")) || Directory.Exists(Path.Combine(output, "traffic")))
            throw new IOException("Traffic partition already exists; use a new sector directory to rebuild.");
        var sourceManifest = await File.ReadAllBytesAsync(Path.Combine(input, "manifest.json"));
        var sourceHash = Convert.ToHexStringLower(SHA256.HashData(sourceManifest));
        var laneIndexBytes = await File.ReadAllBytesAsync(Path.Combine(output, "sector_index.json"));
        using var laneIndex = JsonDocument.Parse(laneIndexBytes);
        if (laneIndex.RootElement.GetProperty("source_manifest_sha256").GetString() != sourceHash)
            throw new InvalidDataException("Metadata and lane partition must have exactly the same source manifest.");
        var lanes = (await ParquetSerializer.DeserializeAsync<LaneRow>(Path.Combine(input, "lanes.parquet"))).Data;
        var sectors = lanes.GroupBy(l => (l.sector_x, l.sector_z)).ToDictionary(g => g.Key, g => g.Select(l => l.id).ToHashSet(StringComparer.Ordinal));
        var byLane = lanes.ToDictionary(l => l.id, l => (l.sector_x, l.sector_z), StringComparer.Ordinal);
        var present = new List<string>(); var missing = new List<string>(); var sourceTables = new Dictionary<string, object>();
        async Task<T[]> Read<T>(string name) where T : class, new()
        {
            var file = Path.Combine(input, name + ".parquet");
            if (!File.Exists(file)) { missing.Add(name); return []; }
            present.Add(name);
            await using var stream = File.OpenRead(file);
            sourceTables[name] = new { bytes = stream.Length, sha256 = Convert.ToHexStringLower(await SHA256.HashDataAsync(stream)) };
            return (await ParquetSerializer.DeserializeAsync<T>(file)).Data.ToArray();
        }
        var conflicts = await Read<ConflictPairRow>("conflict_pairs");
        var controls = await Read<LaneControlRow>("lane_controls");
        var signals = await Read<SignalRow>("signals");
        object? profileResolution = null;
        if(game is not null)
        {
            using var archives = ArchiveSet.Open(new Options("inspect",game,output,mods,[],false));
            using var sourceDocument=JsonDocument.Parse(sourceManifest);
            if(sourceDocument.RootElement.GetProperty("game_version").GetString()!=archives.Version)throw new InvalidDataException("Profile game version differs from source map.");
            var expected=JsonSerializer.Deserialize<List<ArchiveInfo>>(await File.ReadAllTextAsync(Path.Combine(input,"archives.json"))) ?? throw new InvalidDataException("Missing archive inventory.");
            if(expected.Count!=archives.Inventory.Count || expected.Zip(archives.Inventory).Any(p=>p.First.Name!=p.Second.Name||p.First.Bytes!=p.Second.Bytes||p.First.ModifiedUtc!=p.Second.ModifiedUtc))
                throw new InvalidDataException("Installed archive order/fingerprints differ from original map export.");
            var resolver=SignalProfileResolver.Load(archives);
            Console.WriteLine("Reading native prefab overrides and country-border connectivity for signal profiles...");
            var map=TruckLib.ScsMap.Map.Open("/map/europe.mbd",archives.FileSystem);resolver.BindMap(map);
            signals=signals.Select(resolver.Resolve).ToArray();
            var proof=JsonSerializer.SerializeToUtf8Bytes(resolver.Document(archives.Version),Program.Json);
            await File.WriteAllBytesAsync(Path.Combine(output,"signal-profile-evidence.json"),proof);
            await ParquetSerializer.SerializeAsync(signals,Path.Combine(output,"signals-resolved.parquet"));
            await using var resolvedFile=File.OpenRead(Path.Combine(output,"signals-resolved.parquet"));
            var resolvedHash=Convert.ToHexStringLower(await SHA256.HashDataAsync(resolvedFile));
            profileResolution=new { evidence_path="signal-profile-evidence.json", evidence_sha256=Convert.ToHexStringLower(SHA256.HashData(proof)), evidence_bytes=proof.Length,
                resolved_signals_path="signals-resolved.parquet", resolved_signals_sha256=resolvedHash, resolved_signals_bytes=resolvedFile.Length,
                types=signals.GroupBy(s=>s.type).ToDictionary(g=>g.Key,g=>g.Count()),
                statuses=signals.GroupBy(s=>s.profile_resolution ?? "unknown").ToDictionary(g=>g.Key,g=>g.Count()),
                activation_verified=false, geometry_permissions_changed=false };
            Console.WriteLine(JsonSerializer.Serialize(profileResolution,Program.Json));
        }
        var signalLanes = await Read<SignalLaneRow>("signal_lanes");
        var stops = await Read<StopRow>("stop_lines");
        foreach (var id in conflicts.SelectMany(c => new[] { c.lane_a, c.lane_b }).Concat(controls.Select(c => c.lane_id))
            .Concat(signalLanes.Select(s => s.lane_id)).Concat(stops.Select(s => s.lane_id)))
            if (!byLane.ContainsKey(id)) throw new InvalidDataException($"Annotation refers to unknown source lane {id}.");
        var conflictSectors = conflicts.SelectMany(c => new[] { byLane[c.lane_a], byLane[c.lane_b] }.Distinct().Select(s => (Sector: s, Row: c))).ToLookup(x => x.Sector, x => x.Row);
        var controlSectors = controls.ToLookup(c => byLane[c.lane_id]);
        var signalLaneSectors = signalLanes.ToLookup(c => byLane[c.lane_id]);
        var stopSectors = stops.ToLookup(c => byLane[c.lane_id]);
        var signalsById = signals.ToLookup(s => s.id);
        Directory.CreateDirectory(Path.Combine(output, "traffic"));
        var descriptors = new List<object>(); long total = 0;
        foreach (var descriptor in laneIndex.RootElement.GetProperty("shards").EnumerateArray())
        {
            var x = descriptor.GetProperty("sector_x").GetInt32(); var z = descriptor.GetProperty("sector_z").GetInt32();
            var key = (x, z); var owned = sectors[key]; var pairs = conflictSectors[key].ToArray();
            var links = signalLaneSectors[key].ToArray(); var localControls = controlSectors[key].ToArray(); var localStops = stopSectors[key].ToArray();
            // Include external conflict partners as reference identities, even
            // when their geometry lies beyond this loaded window.
            var referenceIds = owned.Concat(pairs.SelectMany(p => new[] { p.lane_a, p.lane_b })).Distinct().Order(StringComparer.Ordinal).ToArray();
            var signalIds = links.Select(l => l.signal_id).Concat(localControls.Select(c => c.signal_group_id).OfType<string>())
                .Concat(localStops.Select(s => s.signal_id).OfType<string>()).Distinct().ToArray();
            var document = new { schema_version = 1, source_lane_ids = referenceIds,
                reference_only_lane_ids = referenceIds.Where(id => !owned.Contains(id)).ToArray(),
                annotations_present = missing.Count == 0, conflicts = pairs, lane_controls = localControls,
                signals = signalIds.SelectMany(id => signalsById[id]).ToArray(), signal_lanes = links, stop_lines = localStops };
            var bytes = JsonSerializer.SerializeToUtf8Bytes(document);
            var relative = $"traffic/{x}_{z}.json.gz"; var path = Path.Combine(output, relative);
            await using (var file = File.Create(path))
            await using (var gzip = new GZipStream(file, CompressionLevel.Fastest)) await gzip.WriteAsync(bytes);
            await using var compressed = File.OpenRead(path); total += compressed.Length;
            descriptors.Add(new { sector_x = x, sector_z = z, path = relative,
                bounds = descriptor.GetProperty("bounds").Clone(), bytes = compressed.Length,
                uncompressed_bytes = bytes.Length, sha256 = Convert.ToHexStringLower(await SHA256.HashDataAsync(compressed)) });
        }
        var laneById = lanes.ToDictionary(l => l.id, StringComparer.Ordinal);
        var graphLinks = (await ParquetSerializer.DeserializeAsync<LinkRow>(Path.Combine(input, "lane_links.parquet"))).Data;
        var successors = graphLinks.ToLookup(l => l.lane_id, l => l.successor_id);
        var starts = lanes.Where(l => l.node_start != 0).ToLookup(l => l.node_start, l => l.id);
        var nativeNodes = lanes.SelectMany(l => new[] { l.node_start, l.node_end }).Where(n => n != 0).Distinct().Order().ToArray();
        var nativeRelative = "native_route_index.json.gz"; long nativeRawBytes;
        // Collapse only internal zero-endpoint curves. This node graph is a
        // coarse reachability index; runtime still checks exact lane channels.
        using (var raw = new MemoryStream())
        {
            await using (var json = new Utf8JsonWriter(raw))
            {
                json.WriteStartObject(); json.WriteNumber("schema_version", 1);
                json.WriteString("source_manifest_sha256", sourceHash);
                json.WriteString("source_cache_key", laneIndex.RootElement.GetProperty("source_cache_key").GetString());
                json.WriteBoolean("topology_only", true); json.WriteStartArray("nodes");
                foreach (var node in nativeNodes)
                {
                    var seen = new HashSet<string>(StringComparer.Ordinal); var next = new SortedSet<ulong>();
                    var queue = new Queue<string>(starts[node]);
                    while (queue.TryDequeue(out var id))
                    {
                        if (!seen.Add(id)) continue;
                        if (!laneById.TryGetValue(id, out var lane)) throw new InvalidDataException("Dangling global lane connection.");
                        if (lane.node_start != 0 && lane.node_start != node) { next.Add(lane.node_start); continue; }
                        if (lane.node_end != 0) { if (lane.node_end != node) next.Add(lane.node_end); continue; }
                        foreach (var successor in successors[id]) queue.Enqueue(successor);
                    }
                    json.WriteStartObject(); json.WriteNumber("uid", node); json.WriteStartArray("next");
                    foreach (var target in next) json.WriteNumberValue(target);
                    json.WriteEndArray(); json.WriteEndObject();
                }
                json.WriteEndArray(); json.WriteEndObject(); await json.FlushAsync();
            }
            nativeRawBytes = raw.Length; raw.Position = 0;
            await using var file = File.Create(Path.Combine(output, nativeRelative));
            await using var gzip = new GZipStream(file, CompressionLevel.Fastest); await raw.CopyToAsync(gzip);
        }
        object nativeDescriptor;
        await using (var file = File.OpenRead(Path.Combine(output, nativeRelative)))
            nativeDescriptor = new { path = nativeRelative, bytes = file.Length, uncompressed_bytes = nativeRawBytes,
                sha256 = Convert.ToHexStringLower(await SHA256.HashDataAsync(file)), node_count = nativeNodes.Length };
        var index = new { schema_version = 1, format = "etsads_sector_traffic_json_gzip_v1",
            coordinate_system = "ENU", units = "metres", source_manifest_sha256 = sourceHash,
            lane_index_sha256 = Convert.ToHexStringLower(SHA256.HashData(laneIndexBytes)),
            source_cache_key = laneIndex.RootElement.GetProperty("source_cache_key").GetString(),
            present_tables = present, missing_tables = missing, source_tables = sourceTables,
            accepted_for_autonomous_driving = false, compressed_bytes = total, shards = descriptors, native_route_index = nativeDescriptor,
            signal_profile_resolution=profileResolution };
        var temporary = Path.Combine(output, "traffic_index.json.tmp");
        await File.WriteAllTextAsync(temporary, JsonSerializer.Serialize(index, Program.Json));
        File.Move(temporary, Path.Combine(output, "traffic_index.json"));
        Console.WriteLine(JsonSerializer.Serialize(new { output, shards = descriptors.Count, compressed_bytes = total, missing_tables = missing }, Program.Json));
        return 0;
    }
}
