// SPDX-License-Identifier: GPL-2.0-only
using System.Globalization;
using System.Text.Json;
using System.Security.Cryptography;
using System.Text;
using Parquet;
using Parquet.Serialization;

namespace Etsads.MapExport;

internal static class MapSlice
{
    public static async Task<int> Run(string[] args)
    {
        var options = new Dictionary<string, string>(StringComparer.Ordinal);
        for (var i = 1; i < args.Length; i += 2)
        {
            if (i + 1 >= args.Length || args[i] is not ("--input" or "--x" or "--y" or "--z" or "--radius" or "--output"))
                throw new ArgumentException("Expected slice --input PATH --x ENU_X --y ENU_Y --radius METRES --output PATH.");
            options.Add(args[i], args[i + 1]);
        }
        foreach (var key in new[] { "--input", "--x", "--y", "--radius", "--output" })
            if (!options.ContainsKey(key)) throw new ArgumentException($"Missing {key}.");
        var x = double.Parse(options["--x"], CultureInfo.InvariantCulture);
        var y = double.Parse(options["--y"], CultureInfo.InvariantCulture);
        var radius = double.Parse(options["--radius"], CultureInfo.InvariantCulture);
        double? z = options.TryGetValue("--z", out var height) ? double.Parse(height, CultureInfo.InvariantCulture) : null;
        if (!double.IsFinite(x) || !double.IsFinite(y) || z is { } zv && !double.IsFinite(zv)
            || !double.IsFinite(radius) || radius <= 0 || radius > 5000)
            throw new ArgumentException("Coordinates must be finite and radius in (0,5000] metres.");
        var input = Path.GetFullPath(options["--input"]);
        var output = Path.GetFullPath(options["--output"]);
        if (string.Equals(Path.TrimEndingDirectorySeparator(input), Path.TrimEndingDirectorySeparator(output), StringComparison.OrdinalIgnoreCase))
            throw new ArgumentException("Slice output must be separate from the source export.");
        var manifestBytes = await File.ReadAllBytesAsync(Path.Combine(input, "manifest.json"));
        using var manifest = JsonDocument.Parse(manifestBytes);
        if (manifest.RootElement.GetProperty("schema_version").GetInt32() != 1
            || manifest.RootElement.GetProperty("coordinate_system").GetString() != "ENU"
            || manifest.RootElement.GetProperty("units").GetString() != "metres")
            throw new InvalidDataException("Unsupported source schema, coordinates or units.");
        var laneTable = await ParquetSerializer.DeserializeAsync<LaneRow>(Path.Combine(input, "lanes.parquet"));
        var selected = laneTable.Data.Where(lane => lane.min_x <= x + radius && lane.max_x >= x - radius
            && lane.min_y <= y + radius && lane.max_y >= y - radius).ToDictionary(lane => lane.id);
        if (selected.Count == 0) throw new InvalidDataException("No lanes intersect the requested area.");
        var points = selected.Keys.ToDictionary(id => id, _ => new List<(int Index, double X, double Y, double Z)>());
        await using (var reader = await ParquetReader.CreateAsync(Path.Combine(input, "lane_points.parquet")))
        {
            var fields = reader.Schema.GetDataFields().ToDictionary(field => field.Name);
            for (var rowGroup = 0; rowGroup < reader.RowGroupCount; rowGroup++)
            {
                using var group = reader.OpenRowGroupReader(rowGroup);
                var count = checked((int)group.RowCount);
                var ids = new string?[count];
                await group.ReadAsync(fields["lane_id"], ids.AsMemory());
                if (!ids.Any(id => id is not null && selected.ContainsKey(id))) continue;
                var indices = new int[count]; var xs = new double[count]; var ys = new double[count]; var zs = new double[count];
                await group.ReadAsync<int>(fields["point_index"], indices.AsMemory());
                await group.ReadAsync<double>(fields["x"], xs.AsMemory());
                await group.ReadAsync<double>(fields["y"], ys.AsMemory());
                await group.ReadAsync<double>(fields["z"], zs.AsMemory());
                for (var i = 0; i < count; i++) if (ids[i] is { } id && points.TryGetValue(id, out var list)) list.Add((indices[i], xs[i], ys[i], zs[i]));
            }
        }
        foreach (var (id, list) in points)
        {
            list.Sort((a, b) => a.Index.CompareTo(b.Index));
            if (list.Count < 2 || list.Where((point, index) => point.Index != index).Any())
                throw new InvalidDataException($"Incomplete or duplicate point sequence for {id}.");
        }
        var allLinks = await ParquetSerializer.DeserializeAsync<LinkRow>(Path.Combine(input, "lane_links.parquet"));
        var links = allLinks.Data.Where(link => selected.ContainsKey(link.lane_id) && selected.ContainsKey(link.successor_id)).ToArray();
        var boundaryLinks = allLinks.Data.Where(link => selected.ContainsKey(link.lane_id) != selected.ContainsKey(link.successor_id))
            .Select(link => new { link.lane_id, link.successor_id, link.source, link.endpoint_gap_m,
                region_direction = selected.ContainsKey(link.lane_id) ? "outgoing" : "incoming" }).ToArray();
        var successors = links.ToLookup(link => link.lane_id, link => link.successor_id);
        var predecessors = links.ToLookup(link => link.successor_id, link => link.lane_id);
        var allNeighbors = await ReadOptional<NeighborRow>("lane_neighbors");
        var neighborLookup = allNeighbors.Where(link => selected.ContainsKey(link.lane_id))
            .ToDictionary(link => (link.lane_id, link.side));
        if (allNeighbors.Any(link => link.side is not ("left" or "right")))
            throw new InvalidDataException("Unknown lane-neighbor side in source data.");
        var neighbors = allNeighbors.Where(link => selected.ContainsKey(link.lane_id) && selected.ContainsKey(link.neighbor_id)).ToArray();
        var boundaryNeighbors = allNeighbors.Where(link => selected.ContainsKey(link.lane_id) != selected.ContainsKey(link.neighbor_id)).ToArray();
        var conflictPairs = (await ReadOptional<ConflictPairRow>("conflict_pairs")).Where(pair => selected.ContainsKey(pair.lane_a) && selected.ContainsKey(pair.lane_b)).ToArray();
        var controls = (await ReadOptional<LaneControlRow>("lane_controls")).Where(control => selected.ContainsKey(control.lane_id)).ToArray();
        var signalLanes = (await ReadOptional<SignalLaneRow>("signal_lanes")).Where(link => selected.ContainsKey(link.lane_id)).ToArray();
        var signalIds = signalLanes.Select(link => link.signal_id).ToHashSet();
        var signals = (await ReadOptional<SignalRow>("signals")).Where(signal => signalIds.Contains(signal.id)).ToArray();
        var stops = (await ReadOptional<StopRow>("stop_lines")).Where(stop => selected.ContainsKey(stop.lane_id)).ToArray();
        bool InArea(double px, double py) => Math.Pow(px - x, 2) + Math.Pow(py - y, 2) <= radius * radius;
        var nodes = (await ReadOptional<NodeRow>("nodes")).Where(node => InArea(node.x, node.y)).ToArray();
        double Distance(double px, double py, double pz) => Math.Sqrt(Math.Pow(px - x, 2) + Math.Pow(py - y, 2) + (z is { } referenceHeight ? Math.Pow(pz - referenceHeight, 2) : 0));
        var slots = (await ReadOptional<SlotRow>("slots")).Where(slot => InArea(slot.x, slot.y)).OrderBy(slot => Distance(slot.x, slot.y, slot.z)).ToArray();
        var pois = (await ReadOptional<PoiRow>("pois")).Where(poi => InArea(poi.x, poi.y)).OrderBy(poi => Distance(poi.x, poi.y, poi.z)).ToArray();
        var polygonRows = await ReadOptional<PolygonRow>("drivable_polygons");
        var nearbyPolygons = polygonRows.Where(point => InArea(point.x, point.y)).Select(point => point.id).ToHashSet();
        var polygons = polygonRows.Where(point => nearbyPolygons.Contains(point.id)).GroupBy(point => point.id).Select(group => new
        {
            id = group.Key, owner_uid = group.First().owner_uid, geometry_status = group.First().geometry_status,
            physical_boundary_verified = false,
            points = group.OrderBy(point => point.point_index).Select(point => new { x = point.x, y = point.y, z = point.z }).ToArray(),
        }).ToArray();
        var roadCandidates = selected.Values.Where(lane => lane.kind == "road" && lane.ai_truck_access is true)
            .Select(lane =>
            {
                var lanePoints = points[lane.id];
                var nearest = lanePoints.Select((point, index) => (Point: point, Index: index,
                    Distance: Distance(point.X, point.Y, point.Z))).MinBy(value => value.Distance);
                var first = lanePoints[0]; var last = lanePoints[^1];
                var length = lanePoints.Zip(lanePoints.Skip(1)).Sum(pair => Math.Sqrt(Math.Pow(pair.First.X - pair.Second.X, 2)
                    + Math.Pow(pair.First.Y - pair.Second.Y, 2) + Math.Pow(pair.First.Z - pair.Second.Z, 2)));
                var chord = Math.Sqrt(Math.Pow(first.X - last.X, 2) + Math.Pow(first.Y - last.Y, 2) + Math.Pow(first.Z - last.Z, 2));
                var a = lanePoints[Math.Min(nearest.Index, lanePoints.Count - 2)];
                var b = lanePoints[Math.Min(nearest.Index + 1, lanePoints.Count - 1)];
                return new
                {
                    lane_id = lane.id, lane.owner_uid, lane.centerline_validated, lane.geometry_status,
                    lane.speed_class, lane.navigation_width_m, distance_m = nearest.Distance, length_m = length,
                    arc_to_chord_ratio = chord > 1e-3 ? length / chord : 999,
                    nearest_enu = new { x = nearest.Point.X, y = nearest.Point.Y, z = nearest.Point.Z },
                    nearest_scs = new { x = nearest.Point.X, y = nearest.Point.Z, z = -nearest.Point.Y },
                    yaw_enu = Math.Atan2(b.Y - a.Y, b.X - a.X), physical_boundary_verified = false,
                };
            }).Where(candidate => candidate.length_m >= 30 && candidate.arc_to_chord_ratio <= 1.04)
            .OrderBy(candidate => candidate.distance_m).Take(12).ToArray();
        Directory.CreateDirectory(output);
        File.Delete(Path.Combine(output, "manifest.json"));
        await using var stream = File.Create(Path.Combine(output, "runtime-map.json"));
        await using var json = new Utf8JsonWriter(stream);
        json.WriteStartObject();
        json.WriteNumber("schema_version", 1);
        json.WriteString("source", "installed_game_archives");
        json.WriteString("game_version", manifest.RootElement.GetProperty("game_version").GetString());
        json.WriteBoolean("accepted_for_autonomous_driving", false);
        json.WriteBoolean("clipped_to_region", true);
        json.WriteString("source_cache_key", manifest.RootElement.GetProperty("cache_key").GetString());
        // Native item-node positions are route references, not lane-centre endpoints.
        json.WritePropertyName("nodes"); JsonSerializer.Serialize(json, nodes);
        json.WriteStartArray("lanes");
        foreach (var lane in selected.Values.OrderBy(lane => lane.id, StringComparer.Ordinal))
        {
            neighborLookup.TryGetValue((lane.id, "left"), out var leftSource);
            neighborLookup.TryGetValue((lane.id, "right"), out var rightSource);
            var left = leftSource is not null && selected.ContainsKey(leftSource.neighbor_id) ? leftSource.neighbor_id : null;
            var right = rightSource is not null && selected.ContainsKey(rightSource.neighbor_id) ? rightSource.neighbor_id : null;
            JsonSerializer.Serialize(json, new
            {
                lane.id, lane.owner_uid, lane.node_start, lane.node_end,
                center = points[lane.id].Select(point => new { x = point.X, y = point.Y, z = point.Z }),
                width = lane.width_m, speed_limit = lane.speed_limit_mps,
                navigation_width = lane.navigation_width_m, lane.centerline_validated,
                physical_boundaries_verified = false, truck_access = lane.ai_truck_access,
                lane.speed_class, lane.speed_source, lane.geometry_status,
                successors = successors[lane.id].Distinct(), predecessors = predecessors[lane.id].Distinct(),
                left, right,
                can_change_left = left is not null && leftSource?.change_allowed is true,
                can_change_right = right is not null && rightSource?.change_allowed is true,
                left_change_allowed_raw = leftSource?.change_allowed,
                right_change_allowed_raw = rightSource?.change_allowed,
            });
            await json.FlushAsync();
        }
        json.WriteEndArray();
        json.WritePropertyName("lane_neighbors"); JsonSerializer.Serialize(json, neighbors);
        json.WritePropertyName("boundary_neighbors"); JsonSerializer.Serialize(json, boundaryNeighbors);
        json.WritePropertyName("boundary_links"); JsonSerializer.Serialize(json, boundaryLinks);
        json.WritePropertyName("conflicts"); JsonSerializer.Serialize(json, conflictPairs);
        json.WritePropertyName("lane_controls"); JsonSerializer.Serialize(json, controls);
        json.WritePropertyName("signals"); JsonSerializer.Serialize(json, signals);
        json.WritePropertyName("signal_lanes"); JsonSerializer.Serialize(json, signalLanes);
        json.WritePropertyName("stop_lines"); JsonSerializer.Serialize(json, stops);
        json.WritePropertyName("slots"); JsonSerializer.Serialize(json, slots);
        json.WritePropertyName("pois"); JsonSerializer.Serialize(json, pois);
        json.WritePropertyName("boundary_candidates"); JsonSerializer.Serialize(json, polygons);
        json.WriteEndObject();
        await json.FlushAsync();
        await json.DisposeAsync();
        await stream.DisposeAsync();
        var sourceHash = Convert.ToHexStringLower(SHA256.HashData(manifestBytes));
        var identity = sourceHash + FormattableString.Invariant($"|slice-v2|{x:R}|{y:R}|{z:R}|{radius:R}");
        await File.WriteAllTextAsync(Path.Combine(output, "manifest.json"), JsonSerializer.Serialize(new
        {
            schema_version = 1, source = "installed_game_archives", is_region = true,
            slice_format_version = 2,
            game_version = manifest.RootElement.GetProperty("game_version").GetString(),
            game_version_source = "inherited from source export PE ProductVersion; source manifest SHA-256 recorded",
            game_file_version = manifest.RootElement.TryGetProperty("game_file_version", out var fileVersion) ? fileVersion.GetString() : null,
            source_cache_key = manifest.RootElement.GetProperty("cache_key").GetString(), source_manifest_sha256 = sourceHash,
            cache_key = Convert.ToHexStringLower(SHA256.HashData(Encoding.UTF8.GetBytes(identity)))[..12],
            coordinate_system = "ENU", units = "metres", center = new { x, y, z }, radius_m = radius,
            counts = new { lanes = selected.Count, lane_points = points.Values.Sum(list => list.Count), lane_links = links.Length,
                nodes = nodes.Length, lane_neighbors = neighbors.Length, boundary_links = boundaryLinks.Length, boundary_neighbors = boundaryNeighbors.Length,
                conflict_pairs = conflictPairs.Length, lane_controls = controls.Length, signals = signals.Length, stop_lines = stops.Length,
                slots = slots.Length, pois = pois.Length, boundary_candidates = polygons.Length },
            accepted_for_autonomous_driving = false,
            capabilities = manifest.RootElement.TryGetProperty("capabilities", out var capabilities) ? capabilities : (JsonElement?)null,
        }, Program.Json));
        await File.WriteAllTextAsync(Path.Combine(output, "area-assets.json"), JsonSerializer.Serialize(new
        {
            source_cache_key = manifest.RootElement.GetProperty("cache_key").GetString(), reference_enu = new { x, y, z },
            radius_m = radius, note = "Company slots with nonzero node_uid are native map poses. UI polygons are candidates only, never collision boundaries.",
            slots, pois, boundary_candidates = polygons, nearest_low_curvature_roads = roadCandidates,
        }, Program.Json));
        Console.WriteLine(JsonSerializer.Serialize(new { lanes = selected.Count, points = points.Values.Sum(list => list.Count),
            links = links.Length, neighbors = neighbors.Length, boundary_links = boundaryLinks.Length, boundary_neighbors = boundaryNeighbors.Length,
            output = Path.Combine(output, "runtime-map.json"),
            note = "Preserves full-export calibration. Cross-boundary source edges are explicit metadata, never executable graph links or permissions." }, Program.Json));
        return 0;

        async Task<IList<T>> ReadOptional<T>(string name) where T : class, new() => File.Exists(Path.Combine(input, name + ".parquet"))
            ? (await ParquetSerializer.DeserializeAsync<T>(Path.Combine(input, name + ".parquet"))).Data : new List<T>();
    }
}
