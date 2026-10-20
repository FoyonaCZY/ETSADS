// SPDX-License-Identifier: GPL-2.0-only
using System.IO.Compression;
using System.Security.Cryptography;
using System.Text.Json;
using Parquet;
using Parquet.Serialization;

namespace Etsads.MapExport;

/// <summary>Bounded point streaming into independently readable sector shards.
/// Original Parquet files remain untouched. Native IDs and nullable metadata
/// are retained, and graph clipping happens only when a region is assembled.</summary>
internal static class Partition
{
    private sealed class Shard
    {
        public required int X { get; init; }
        public required int Z { get; init; }
        public required string RelativePath { get; init; }
        public double MinX = double.PositiveInfinity, MinY = double.PositiveInfinity;
        public double MaxX = double.NegativeInfinity, MaxY = double.NegativeInfinity;
        public long Lanes, Points, UncompressedBytes;
    }
    private sealed class StreamPool(string root) : IDisposable
    {
        private readonly Dictionary<string, (GZipStream Stream, long Tick)> open = [];
        private long tick;
        public void Write(Shard shard, byte[] line)
        {
            if (!open.TryGetValue(shard.RelativePath, out var active))
            {
                if (open.Count >= 16)
                {
                    var oldest = open.MinBy(entry => entry.Value.Tick);
                    oldest.Value.Stream.Dispose(); open.Remove(oldest.Key);
                }
                var file = new FileStream(Path.Combine(root, shard.RelativePath), FileMode.Append, FileAccess.Write, FileShare.Read, 65536);
                active = (new GZipStream(file, CompressionLevel.Fastest), 0);
            }
            active.Stream.Write(line); active.Stream.WriteByte(10);
            open[shard.RelativePath] = (active.Stream, ++tick);
            shard.UncompressedBytes += line.Length + 1;
        }
        public void Dispose() { foreach (var item in open.Values) item.Stream.Dispose(); open.Clear(); }
    }
    public static async Task<int> Run(string[] args)
    {
        if (args.Length != 5 || args[1] != "--input" || args[3] != "--output")
            throw new ArgumentException("Expected partition --input PARQUET_EXPORT --output NEW_DIRECTORY.");
        var input = Path.GetFullPath(args[2]); var output = Path.GetFullPath(args[4]);
        if (Directory.Exists(output) || File.Exists(output)) throw new IOException("Partition output must be a new directory.");
        var manifestBytes = await File.ReadAllBytesAsync(Path.Combine(input, "manifest.json"));
        using var manifest = JsonDocument.Parse(manifestBytes);
        if (manifest.RootElement.GetProperty("schema_version").GetInt32() != 1
            || manifest.RootElement.GetProperty("coordinate_system").GetString() != "ENU"
            || manifest.RootElement.GetProperty("units").GetString() != "metres")
            throw new InvalidDataException("Unsupported source schema, coordinates or units.");
        var clock = System.Diagnostics.Stopwatch.StartNew();
        var lanes = (await ParquetSerializer.DeserializeAsync<LaneRow>(Path.Combine(input, "lanes.parquet"))).Data.ToDictionary(l => l.id, StringComparer.Ordinal);
        var links = (await ParquetSerializer.DeserializeAsync<LinkRow>(Path.Combine(input, "lane_links.parquet"))).Data;
        var successors = links.ToLookup(l => l.lane_id, l => l.successor_id);
        var predecessors = links.ToLookup(l => l.successor_id, l => l.lane_id);
        var neighbors = File.Exists(Path.Combine(input, "lane_neighbors.parquet"))
            ? (await ParquetSerializer.DeserializeAsync<NeighborRow>(Path.Combine(input, "lane_neighbors.parquet"))).Data : [];
        var neighborLookup = neighbors.ToDictionary(n => (n.lane_id, n.side));
        if (neighbors.Any(n => n.side is not ("left" or "right") || !lanes.ContainsKey(n.neighbor_id) || !lanes.ContainsKey(n.lane_id))
            || links.Any(l => !lanes.ContainsKey(l.lane_id) || !lanes.ContainsKey(l.successor_id)))
            throw new InvalidDataException("Source has malformed neighbor metadata or unresolved graph links.");
        Directory.CreateDirectory(Path.Combine(output, "sectors"));
        var shards = new Dictionary<(int X, int Z), Shard>();
        var seen = new HashSet<string>(StringComparer.Ordinal);
        var center = new List<(double X, double Y, double Z)>();
        string? current = null; long pointCount = 0;
        using (var pool = new StreamPool(output))
        {
            void FinishLane()
            {
                if (current is null) return;
                if (!lanes.TryGetValue(current, out var lane) || !seen.Add(current) || center.Count < 2)
                    throw new InvalidDataException($"Unknown, repeated, or incomplete lane points: {current}.");
                if (!shards.TryGetValue((lane.sector_x, lane.sector_z), out var shard))
                    shards[(lane.sector_x, lane.sector_z)] = shard = new Shard { X = lane.sector_x, Z = lane.sector_z,
                        RelativePath = $"sectors/{lane.sector_x}_{lane.sector_z}.lanes.jsonl.gz" };
                neighborLookup.TryGetValue((lane.id, "left"), out var left);
                neighborLookup.TryGetValue((lane.id, "right"), out var right);
                var row = new
                {
                    bounds = new[] { lane.min_x, lane.min_y, lane.max_x, lane.max_y },
                    // Full source metadata remains available for future readers.
                    source_row = lane,
                    lane = new
                    {
                        lane.id, lane.owner_uid, lane.node_start, lane.node_end,
                        center = center.Select(p => new { x = p.X, y = p.Y, z = p.Z }).ToArray(),
                        width = lane.width_m, speed_limit = lane.speed_limit_mps, navigation_width = lane.navigation_width_m,
                        lane.geometry_status, lane.centerline_validated, physical_boundaries_verified = false,
                        truck_access = lane.ai_truck_access, lane.speed_class, lane.speed_source,
                        successors = successors[lane.id].Distinct().ToArray(), predecessors = predecessors[lane.id].Distinct().ToArray(),
                        left = left?.neighbor_id, right = right?.neighbor_id,
                        can_change_left = left?.change_allowed is true, can_change_right = right?.change_allowed is true,
                        left_change_allowed_raw = left?.change_allowed, right_change_allowed_raw = right?.change_allowed,
                    },
                };
                pool.Write(shard, JsonSerializer.SerializeToUtf8Bytes(row));
                shard.Lanes++; shard.Points += center.Count;
                shard.MinX = Math.Min(shard.MinX, lane.min_x); shard.MinY = Math.Min(shard.MinY, lane.min_y);
                shard.MaxX = Math.Max(shard.MaxX, lane.max_x); shard.MaxY = Math.Max(shard.MaxY, lane.max_y);
                center.Clear();
            }
            await using var reader = await ParquetReader.CreateAsync(Path.Combine(input, "lane_points.parquet"));
            var fields = reader.Schema.GetDataFields().ToDictionary(f => f.Name);
            for (var rowGroup = 0; rowGroup < reader.RowGroupCount; rowGroup++)
            {
                using var group = reader.OpenRowGroupReader(rowGroup);
                var count = checked((int)group.RowCount);
                var ids = new string?[count]; var indices = new int[count];
                var xs = new double[count]; var ys = new double[count]; var zs = new double[count];
                await group.ReadAsync(fields["lane_id"], ids.AsMemory());
                await group.ReadAsync<int>(fields["point_index"], indices.AsMemory());
                await group.ReadAsync<double>(fields["x"], xs.AsMemory());
                await group.ReadAsync<double>(fields["y"], ys.AsMemory());
                await group.ReadAsync<double>(fields["z"], zs.AsMemory());
                for (var i = 0; i < count; i++)
                {
                    if (ids[i] is null || !double.IsFinite(xs[i]) || !double.IsFinite(ys[i]) || !double.IsFinite(zs[i]))
                        throw new InvalidDataException("Invalid point record.");
                    if (ids[i] != current) { FinishLane(); current = ids[i]; }
                    if (indices[i] != center.Count) throw new InvalidDataException($"Noncontiguous point index for {current}.");
                    center.Add((xs[i], ys[i], zs[i])); pointCount++;
                }
                if ((rowGroup + 1) % 40 == 0) Console.WriteLine($"Partitioned {pointCount:N0} points; {seen.Count:N0} complete lanes.");
            }
            FinishLane();
        }
        if (seen.Count != lanes.Count) throw new InvalidDataException("Not every metadata lane has complete points.");
        var sourceHash = Convert.ToHexStringLower(SHA256.HashData(manifestBytes));
        var sourceTables = new Dictionary<string, object>();
        foreach (var name in new[] { "lanes.parquet", "lane_points.parquet", "lane_links.parquet", "lane_neighbors.parquet" })
        {
            var path = Path.Combine(input, name);
            if (!File.Exists(path)) continue;
            await using var file = File.OpenRead(path);
            sourceTables[name] = new { bytes = file.Length, sha256 = Convert.ToHexStringLower(await SHA256.HashDataAsync(file)) };
        }
        var indexShards = new List<object>(); long compressedTotal = 0;
        foreach (var shard in shards.Values.OrderBy(s => s.X).ThenBy(s => s.Z))
        {
            await using var file = File.OpenRead(Path.Combine(output, shard.RelativePath)); compressedTotal += file.Length;
            indexShards.Add(new { sector_x = shard.X, sector_z = shard.Z, path = shard.RelativePath,
                bounds = new[] { shard.MinX, shard.MinY, shard.MaxX, shard.MaxY },
                lane_count = shard.Lanes, point_count = shard.Points, bytes = file.Length,
                uncompressed_bytes = shard.UncompressedBytes, sha256 = Convert.ToHexStringLower(await SHA256.HashDataAsync(file)) });
        }
        var outputManifest = manifest.RootElement.EnumerateObject().ToDictionary(p => p.Name, p => p.Value.Clone());
        outputManifest["storage_layout"] = JsonSerializer.SerializeToElement("sector_jsonl_gzip_v1");
        outputManifest["source_manifest_sha256"] = JsonSerializer.SerializeToElement(sourceHash);
        outputManifest["source_cache_key"] = manifest.RootElement.GetProperty("cache_key").Clone();
        outputManifest["partitioned_tables"] = JsonSerializer.SerializeToElement(new[] { "lanes", "lane_points", "lane_links", "lane_neighbors" });
        outputManifest["scope"] = JsonSerializer.SerializeToElement("Lane graph only. Other original Parquet tables remain in the source export; no inferred widths/speeds or driving acceptance.");
        await File.WriteAllTextAsync(Path.Combine(output, "manifest.json"), JsonSerializer.Serialize(outputManifest, Program.Json));
        var index = new { schema_version = 1, format = "etsads_sector_lanes_jsonl_gzip_v1", coordinate_system = "ENU", units = "metres",
            sector_size_m = 4000, source = manifest.RootElement.GetProperty("source").GetString(),
            accepted_for_autonomous_driving = manifest.RootElement.GetProperty("accepted_for_autonomous_driving").GetBoolean(),
            source_manifest_sha256 = sourceHash, source_cache_key = manifest.RootElement.GetProperty("cache_key").GetString(),
            source_tables = sourceTables, lane_count = lanes.Count, point_count = pointCount, compressed_bytes = compressedTotal,
            elapsed_seconds = clock.Elapsed.TotalSeconds, shards = indexShards };
        var temporary = Path.Combine(output, "sector_index.json.tmp");
        await File.WriteAllTextAsync(temporary, JsonSerializer.Serialize(index, Program.Json));
        File.Move(temporary, Path.Combine(output, "sector_index.json")); // Last, atomic completion marker.
        Console.WriteLine(JsonSerializer.Serialize(new { output, lanes = lanes.Count, points = pointCount, shards = shards.Count,
            compressed_bytes = compressedTotal, elapsed_seconds = clock.Elapsed.TotalSeconds }, Program.Json));
        return 0;
    }
}
