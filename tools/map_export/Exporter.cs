// SPDX-License-Identifier: GPL-2.0-only
using System.Numerics;
using System.Text.Json;
using Parquet;
using Parquet.Data;
using Parquet.Schema;
using Parquet.Serialization;
using TruckLib;
using TruckLib.Models.Ppd;
using TruckLib.ScsMap;
using TruckLib.Sii;

namespace Etsads.MapExport;

// Records use schema spelling deliberately; u64 native IDs remain lossless Parquet unsigned integers.
public sealed record NodeRow(ulong uid, double x, double y, double z)
{
    public NodeRow() : this(0, 0, 0, 0) { }
}
public sealed record LaneRow(string id, ulong owner_uid, ulong node_start, ulong node_end, string kind,
    float? width_m, float? speed_limit_mps, string rule, string geometry_status,
    int sector_x, int sector_z, double min_x, double min_y, double min_z, double max_x, double max_y, double max_z,
    float? navigation_width_m, bool centerline_validated, bool? ai_truck_access, string speed_class, string speed_source)
{
    public LaneRow() : this("", 0, 0, 0, "", null, null, "", "", 0, 0, 0, 0, 0, 0, 0, 0, null, false, null, "", "") { }
}
public sealed record LinkRow(string lane_id, string successor_id, string source, double endpoint_gap_m)
{
    public LinkRow() : this("", "", "", 0) { }
}
public sealed record NeighborRow(string lane_id, string neighbor_id, string side, bool? change_allowed)
{
    public NeighborRow() : this("", "", "", null) { }
}
public sealed record SignalRow(string id, ulong owner_uid, uint semaphore_id, string type, double x, double y,
    double z, double yaw, string profile, float cycle_delay_s, float interval_0, float interval_1, float interval_2, float interval_3)
{
    public SignalRow() : this("", 0, 0, "", 0, 0, 0, 0, "", 0, 0, 0, 0, 0) { }
    public string? raw_type { get; init; }
    public string? raw_profile { get; init; }
    public string? effective_profile { get; init; }
    public string? profile_unit { get; init; }
    public string? profile_country { get; init; }
    public string? profile_resolution { get; init; }
    public string? profile_evidence_id { get; init; }
    public bool timing_resolved { get; init; }
    public uint? resolved_id_map { get; init; }
}
public sealed record SignalLaneRow(string signal_id, string lane_id)
{
    public SignalLaneRow() : this("", "") { }
}
public sealed record SlotRow(string id, ulong owner_uid, ulong node_uid, string kind, string company, string city,
    double x, double y, double z, double yaw)
{
    public SlotRow() : this("", 0, 0, "", "", "", 0, 0, 0, 0) { }
}
public sealed record ConflictRow(string id, ulong owner_uid, string lane_id, float curve_position,
    float radius_m, uint flags, string interpretation);
public sealed record StopRow(string id, string lane_id, string? signal_id, string rule, double x, double y,
    double z, string geometry_status, string control_kind)
{
    public StopRow() : this("", "", null, "", 0, 0, 0, "", "unknown") { }
}
public sealed record PolygonRow(string id, ulong owner_uid, int point_index, double x, double y, double z, string geometry_status)
{
    public PolygonRow() : this("", 0, 0, 0, 0, 0, "") { }
}
public sealed record PoiRow(string id, ulong owner_uid, string kind, double x, double y, double z)
{
    public PoiRow() : this("", 0, "", 0, 0, 0) { }
}
internal sealed record Endpoint(string LaneId, ulong Owner, ulong Node, Vector3 Position, Vector3 Direction);
internal sealed record RoadOffsetObservation(string road_look, string variant, string side, int lane_index,
    double measured_offset_m, double estimated_offset_m, ulong road_uid, ulong node_uid);

internal sealed class Exporter(IFileSystem fs, Definitions definitions)
{
    private readonly List<LaneRow> lanes = [];
    private readonly List<LinkRow> links = [];
    private readonly List<NeighborRow> neighbors = [];
    private readonly List<SignalRow> signals = [];
    private readonly List<SignalLaneRow> signalLanes = [];
    private readonly List<SlotRow> slots = [];
    private readonly List<(SlotRow Slot, ulong Prefab)> companySlots = [];
    private readonly List<ConflictRow> conflicts = [];
    private readonly List<ConflictPairRow> conflictPairs = [];
    private readonly List<LaneControlRow> laneControls = [];
    private readonly List<StopRow> stops = [];
    private readonly List<PolygonRow> polygons = [];
    private readonly List<PoiRow> pois = [];
    private readonly List<Endpoint> starts = [];
    private readonly List<Endpoint> ends = [];
    private readonly Dictionary<string, PrefabDescriptor> descriptorCache = [];
    public Dictionary<string, long> Counts { get; } = [];
    public List<string> Warnings { get; } = [];
    public Dictionary<string, object> Validation { get; } = [];
    public bool WriteRuntimeJson { get; set; }
    public SignalProfileResolver? ProfileResolver { get; set; }
    public string GameVersion { get; set; } = "unknown";
    public HashSet<SectorCoordinate> BaseMapSectors { get; set; } = [];
    private StreamWriter? runtimeGeometry;
    private readonly Dictionary<ulong, object> junctionSamples = [];
    private readonly Dictionary<(string Look, string Variant, string Side, int Index), double> roadCalibration = [];
    private readonly Dictionary<(ulong Road, string Side, int Index), double> roadInstanceCalibration = [];
    private readonly List<(double Length, string LaneId, object Lane, object Metadata)> highwayCandidates = [];

    public async Task Export(Map map, string directory)
    {
        ProfileResolver?.BindMap(map);
        await using var pointWriter = await PointWriter.Create(Path.Combine(directory, "lane_points.parquet"));
        var roads = 0;
        var prefabs = 0;
        var skipped = 0;
        var lastProgress = 0;
        var roadAuditDone = false;
        var maxResidual = 0d;
        using var runtimeStream = WriteRuntimeJson ? File.CreateText(Path.Combine(directory, "runtime-geometry.jsonl")) : null;
        runtimeGeometry = runtimeStream;
        // Prefab boundary points are the native reference used to calibrate road-look centre offsets.
        foreach (var item in map.MapItems.Values.OrderBy(item => item is Prefab ? 0 : item is Road ? 1 : 2).ThenBy(item => item.Uid))
        {
            switch (item)
            {
                case Road road:
                    if (!roadAuditDone) { await WriteRoadAudit(map, directory); roadAuditDone = true; }
                    if (!definitions.RoadLooks.TryGetValue(road.RoadType.String, out var look))
                    { Warn($"Missing road look {road.RoadType.String}"); skipped++; continue; }
                    await AddRoad(road, look, pointWriter);
                    roads++;
                    break;
                case Prefab prefab:
                    if (!definitions.Prefabs.TryGetValue(prefab.Model.String, out var path))
                    { Warn($"Missing prefab definition {prefab.Model.String}"); skipped++; continue; }
                    try
                    {
                        if (!descriptorCache.TryGetValue(path, out var descriptor))
                        {
                            descriptor = PrefabDescriptor.Open(path, fs);
                            descriptorCache[path] = descriptor;
                        }
                        if (descriptor.Nodes.Count == 0) continue;
                        var transform = new PrefabTransform(prefab, descriptor);
                        maxResidual = Math.Max(maxResidual, transform.MaxResidual);
                        await AddPrefab(prefab, descriptor, transform, pointWriter);
                        prefabs++;
                    }
                    catch (Exception ex) when (ex is InvalidDataException or FileNotFoundException or NotSupportedException)
                    { Warn($"Prefab {prefab.Uid}: {ex.Message}"); skipped++; }
                    break;
                case Company company:
                    foreach (var point in company.SpawnPoints)
                    {
                        var p = Geometry.Enu(point.Node.Position);
                        var slot = new SlotRow($"{company.Uid}:company:{point.Node.Uid}", company.Uid, point.Node.Uid,
                            $"{point.DepotType}/{point.Difficulty}", company.CompanyName.String, company.CityName.String,
                            p.X, p.Y, p.Z, Geometry.Yaw(point.Node.Rotation));
                        slots.Add(slot);
                        companySlots.Add((slot, company.Prefab.Uid));
                    }
                    break;
            }
            if (roads + prefabs >= lastProgress + 10000)
            {
                Console.WriteLine($"Geometry: {roads} roads, {prefabs} prefabs, {lanes.Count} lanes.");
                lastProgress = roads + prefabs;
            }
        }
        BuildLinks();
        if (!roadAuditDone) await WriteRoadAudit(map, directory);
        await WriteSlotAudit(directory);
        if(ProfileResolver is not null) await File.WriteAllTextAsync(Path.Combine(directory,"signal-profile-evidence.json"),JsonSerializer.Serialize(ProfileResolver.Document(GameVersion),Program.Json));
        var candidateDirectory = Path.Combine(directory, "highway-candidates");
        Directory.CreateDirectory(candidateDirectory);
        foreach (var candidate in highwayCandidates)
            await File.WriteAllTextAsync(Path.Combine(candidateDirectory, candidate.LaneId.Replace(':', '_') + ".json"),
                JsonSerializer.Serialize(new { schema_version = 1, source = "installed_game_archives",
                    accepted_for_autonomous_driving = false, clipped_to_candidate = true,
                    metadata = candidate.Metadata, lanes = new[] { candidate.Lane } }, Program.Json));
        await File.WriteAllTextAsync(Path.Combine(directory, "highway-candidates.json"),
            JsonSerializer.Serialize(highwayCandidates.Select(c => new { file = "highway-candidates/" + c.LaneId.Replace(':', '_') + ".json", metadata = c.Metadata }), Program.Json));
        await File.WriteAllTextAsync(Path.Combine(directory, "junction-samples.json"), JsonSerializer.Serialize(new
        {
            sample_method = "20 smallest deterministic multiplicative UID hashes among prefabs with at least 3 nodes and 6 nav curves",
            scope = "numerical connectivity plus coordinates for subsequent game checks; visual acceptance pending",
            samples = junctionSamples.OrderBy(s => unchecked(s.Key * 11400714819323198485UL)).Select(s => s.Value),
        }, Program.Json));
        if (runtimeStream is not null)
        {
            await runtimeStream.FlushAsync();
            runtimeStream.Dispose();
            runtimeGeometry = null;
            await WriteRuntime(directory);
            File.Delete(Path.Combine(directory, "runtime-geometry.jsonl"));
        }
        await pointWriter.Flush();
        Counts["lane_points"] = pointWriter.Count;
        await Write("nodes", map.Nodes.Values.Select(n => { var p = Geometry.Enu(n.Position); return new NodeRow(n.Uid, p.X, p.Y, p.Z); }), directory);
        await Write("lanes", lanes, directory);
        await Write("lane_links", links, directory);
        await Write("lane_neighbors", neighbors, directory);
        await Write("signals", signals, directory);
        await Write("signal_lanes", signalLanes, directory);
        await Write("slots", slots, directory);
        await Write("conflicts", conflicts, directory);
        await Write("conflict_pairs", conflictPairs, directory);
        await Write("lane_controls", laneControls, directory);
        await Write("stop_lines", stops, directory);
        await Write("drivable_polygons", polygons, directory);
        await Write("pois", pois, directory);
        Counts["roads_processed"] = roads;
        Counts["prefabs_processed"] = prefabs;
        Counts["items_skipped"] = skipped;
        var ids = lanes.Select(l => l.id).ToHashSet(StringComparer.Ordinal);
        Validation["duplicate_lane_ids"] = lanes.Count - ids.Count;
        Validation["dangling_links"] = links.Count(l => !ids.Contains(l.lane_id) || !ids.Contains(l.successor_id));
        Validation["maximum_prefab_control_node_residual_m"] = maxResidual;
        Validation["link_count"] = links.Count;
        Validation["maximum_link_gap_m"] = links.Count > 0 ? links.Max(l => l.endpoint_gap_m) : 0;
        Validation["road_geometry"] = "Centre offsets calibrated to native prefab boundaries when support and dispersion permit; other offsets estimated. Physical lane-edge widths unverified.";
        Validation["native_conflict_pairs"] = conflictPairs.Count(pair => pair.source == "native_ppd_group");
        Validation["derived_conflict_pairs"] = conflictPairs.Count(pair => pair.source == "derived_geometry");
        Validation["unknown_priority_conflict_pairs"] = conflictPairs.Count(pair => pair.relation == "unknown");
        Validation["conflicts"] = "Native sibling groups decoded against SCS layout and geometric consistency checked; missing/invalid groups use labelled geometric fallback. Unknown priority requires conservative wait.";
        Warn("Speed limits are null until sign/country/truck rules are resolved; consumers must not substitute an unbounded limit.");
        Warn("Drivable polygons contain map display geometry only, not certified collision surface boundaries.");
        Warn("Stop line positions are navigation curve starts, not painted stop-line extraction.");
    }

    private void Warn(string message) { if (!Warnings.Contains(message, StringComparer.Ordinal) && Warnings.Count < 250) Warnings.Add(message); }

    private async Task AddRoad(Road road, Unit look, PointWriter writer)
    {
        foreach (var side in new[] { "left", "right" })
        {
            var placedSide = side == "left" ? road.Left : road.Right;
            var laneRules = definitions.ResolveLanes(look, side, placedSide);
            var policies = laneRules.Select(rule => definitions.LanePolicy(Definitions.Text(rule))).ToArray();
            var offsets = Definitions.Array(look, "lane_offsets_" + side);
            // SCS documents a 4.5 m navigation lane. Road-size/shoulder parameters describe the model,
            // not this nominal lane pitch. Actual centres are still cross-checked against PPD boundaries.
            const float width = 4.5f;
            var reverse = (side == "left") != road.LeftHandTraffic;
            for (var i = 0; i < laneRules.Length; i++)
            {
                var correction = i < offsets.Length && offsets[i] is Vector2 v ? v.X : 0;
                var offset = Definitions.Number(look, "road_offset") + width * (i + 0.5f) + correction;
                var variant = (side == "left" ? road.Left : road.Right).Variant.String;
                var calibrated = roadCalibration.TryGetValue((road.RoadType.String, variant, side, i), out var measured)
                    || roadCalibration.TryGetValue((road.RoadType.String, "*", side, i), out measured);
                var instance = !calibrated && roadInstanceCalibration.TryGetValue((road.Uid, side, i), out measured);
                if (calibrated || instance) offset = (float)measured;
                if (side == "left") offset = -offset;
                var points = Geometry.Road(road, offset);
                if (reverse) points.Reverse();
                var id = $"{road.Uid}:road:{side}:{i}";
                await AddLane(id, road.Uid, reverse ? road.ForwardNode.Uid : road.Node.Uid,
                    reverse ? road.Node.Uid : road.ForwardNode.Uid, "road", null,
                    Definitions.Text(laneRules[i]),
                    calibrated ? "prefab_boundary_calibrated" : instance ? "prefab_endpoint_instance" : "nominal_road_look",
                    points, writer, policies[i], calibrated || instance);
                if (calibrated) Counts["road_lanes_calibrated"] = Counts.GetValueOrDefault("road_lanes_calibrated") + 1;
                if (instance) Counts["road_lanes_instance_calibrated"] = Counts.GetValueOrDefault("road_lanes_instance_calibrated") + 1;
                if (calibrated && policies[i].TruckAccess is true && policies[i].SpeedClass == "motorway"
                    && BaseMapSectors.Contains(Map.GetSectorOfCoordinate(road.Node.Position))
                    && BaseMapSectors.Contains(Map.GetSectorOfCoordinate(road.ForwardNode.Position))
                    && !road.LeftHandTraffic && side == "right" && i == laneRules.Length - 1)
                {
                    var length = points.Zip(points.Skip(1)).Sum(pair => (double)Vector3.Distance(pair.First, pair.Second));
                    var chord = Vector3.Distance(points[0], points[^1]);
                    if (length > 200 && length / chord < 1.01 && Math.Abs(points[0].Z - points[^1].Z) / length < 0.025)
                    {
                        var p = points[0];
                        var metadata = new
                        {
                            road_uid = road.Uid.ToString(), lane_id = id, road_look = road.RoadType.String,
                            length_m = length, arc_to_chord_ratio = length / chord,
                            scs_start = new { x = p.X, y = p.Z, z = -p.Y },
                            enu_start = new { x = p.X, y = p.Y, z = p.Z },
                            sector_x = (int)Math.Floor(p.X / 4000), sector_z = (int)Math.Floor(-p.Y / 4000),
                            navigation_width_m = 4.5, physical_boundaries_verified = false,
                            verification = "base-map, right-hand-traffic, motorway class, native-boundary-calibrated centre, low curvature and grade; in-game review pending",
                        };
                        highwayCandidates.Add((length, id, new
                        {
                            id, owner_uid = road.Uid, node_start = road.Node.Uid, node_end = road.ForwardNode.Uid,
                            center = points.Select(point => new { x = point.X, y = point.Y, z = point.Z }).ToArray(),
                            width = (float?)null, speed_limit = (float?)null, navigation_width = 4.5f,
                            centerline_validated = true, physical_boundaries_verified = false,
                            truck_access = true, speed_class = "motorway", speed_source = "runtime_telemetry_required",
                            geometry_status = "prefab_boundary_calibrated", successors = System.Array.Empty<string>(), predecessors = System.Array.Empty<string>(),
                        }, metadata));
                        highwayCandidates.Sort((a, b) => b.Length.CompareTo(a.Length));
                        if (highwayCandidates.Count > 3) highwayCandidates.RemoveAt(3);
                    }
                }
                if (i + 1 < laneRules.Length)
                {
                    var other = $"{road.Uid}:road:{side}:{i + 1}";
                    // Outwards from the median is driver-right for both RHT carriageways, driver-left for LHT.
                    bool? Change(int from, int to) => placedSide.VariantOverrides.Count > 0 ? null
                        : policies[from].TruckAccess is true && policies[to].TruckAccess is true
                            && policies[from].ChangeOut is true && policies[to].ChangeIn is true;
                    neighbors.Add(new(id, other, road.LeftHandTraffic ? "left" : "right", Change(i, i + 1)));
                    neighbors.Add(new(other, id, road.LeftHandTraffic ? "right" : "left", Change(i + 1, i)));
                }
            }
        }
    }

    private async Task AddPrefab(Prefab prefab, PrefabDescriptor descriptor, PrefabTransform transform, PointWriter writer)
    {
        ulong NodeUid(int descriptorIndex) => prefab.Nodes[(descriptorIndex - prefab.Origin + prefab.Nodes.Count) % prefab.Nodes.Count].Uid;
        var curvePoints = new Dictionary<int, List<Vector3>>();
        for (var i = 0; i < descriptor.NavCurves.Count; i++)
        {
            var curve = descriptor.NavCurves[i];
            var points = Geometry.PrefabCurve(curve, transform);
            curvePoints[i] = points;
            ulong start = 0, end = 0;
            for (var n = 0; n < descriptor.Nodes.Count && n < prefab.Nodes.Count; n++)
            {
                if (descriptor.Nodes[n].InputLines.Contains(i)) start = NodeUid(n);
                if (descriptor.Nodes[n].OutputLines.Contains(i)) end = NodeUid(n);
            }
            var id = $"{prefab.Uid}:prefab:{i}";
            await AddLane(id, prefab.Uid, start, end, "prefab", null, curve.TrafficRule.String,
                "native_ppd_control_nodes_checked", points, writer,
                new LanePolicy(curve.AllowedVehicles is AllowedVehicles.AllVehicles or AllowedVehicles.LargeVehicles, null, null, "unknown"),
                transform.MaxResidual <= 0.3);
            if (curve.SemaphoreId >= 0)
            {
                var signal = $"{prefab.Uid}:signal:{curve.SemaphoreId}";
                signalLanes.Add(new(signal, id));
                stops.Add(new($"{id}:stop", id, signal, curve.TrafficRule.String,
                    points[0].X, points[0].Y, points[0].Z, "curve_start_proxy", "signal"));
            }
        }
        var conflictResult = ConflictBuilder.Build(prefab.Uid, descriptor, curvePoints,
            (curve, t) => Geometry.PrefabPoint(curve, transform.Apply, t), definitions);
        conflictPairs.AddRange(conflictResult.Pairs);
        laneControls.AddRange(conflictResult.Controls);
        Counts["native_conflict_record_checks_failed"] = Counts.GetValueOrDefault("native_conflict_record_checks_failed") + conflictResult.InvalidNativeGroups;
        for (var i = 0; i < conflictResult.Controls.Count; i++)
        {
            var control = conflictResult.Controls[i];
            if (control.control_kind is not ("stop" or "yield" or "unknown")) continue;
            var point = curvePoints[i][0];
            stops.Add(new($"{control.lane_id}:entry", control.lane_id, null, control.traffic_rule,
                point.X, point.Y, point.Z, "curve_start_proxy", control.control_kind));
        }
        foreach (var (index, points) in curvePoints)
            foreach (var successor in descriptor.NavCurves[index].NextLines)
                if (curvePoints.TryGetValue(successor, out var next))
                    links.Add(new($"{prefab.Uid}:prefab:{index}", $"{prefab.Uid}:prefab:{successor}",
                        "native_ppd_next_lines", Vector3.Distance(points[^1], next[0])));
        foreach (var semaphore in descriptor.Semaphores)
        {
            var p = Geometry.Enu(transform.Apply(semaphore.Position));
            var raw = new SignalRow($"{prefab.Uid}:signal:{semaphore.SemaphoreId}", prefab.Uid, semaphore.SemaphoreId,
                semaphore.Type.ToString(), p.X, p.Y, p.Z, transform.Yaw(semaphore.Rotation),
                semaphore.Profile.String, semaphore.CycleDelay, semaphore.Intervals.X, semaphore.Intervals.Y,
                semaphore.Intervals.Z, semaphore.Intervals.W);
            signals.Add(ProfileResolver?.Resolve(raw) ?? raw);
        }
        for (var i = 0; i < descriptor.SpawnPoints.Count; i++)
        {
            var spawn = descriptor.SpawnPoints[i];
            var p = Geometry.Enu(transform.Apply(spawn.Position));
            var id = $"{prefab.Uid}:spawn:{i}";
            if (spawn.Type is SpawnPointType.UnloadEasy or SpawnPointType.UnloadMedium or SpawnPointType.UnloadHard
                or SpawnPointType.Trailer or SpawnPointType.LongTrailer or SpawnPointType.Parking or SpawnPointType.CompanyUnload)
                slots.Add(new(id, prefab.Uid, 0, spawn.Type.ToString(), "", "", p.X, p.Y, p.Z, transform.Yaw(spawn.Rotation)));
            else pois.Add(new(id, prefab.Uid, spawn.Type.ToString(), p.X, p.Y, p.Z));
        }
        for (var i = 0; i < descriptor.Intersections.Count; i++)
        {
            var intersection = descriptor.Intersections[i];
            conflicts.Add(new($"{prefab.Uid}:intersection:{i}", prefab.Uid,
                $"{prefab.Uid}:prefab:{intersection.CurveId}", intersection.Position,
                intersection.Radius, intersection.Flags.Bits, "native_ppd_group_member"));
        }
        // These are explicitly tagged display polygons. Using them as physical surface is unsafe.
        var visited = new HashSet<int>();
        for (var i = 0; i < descriptor.MapPoints.Count; i++)
        {
            if (descriptor.MapPoints[i].RoadSize != RoadSize.Polygon || visited.Contains(i)) continue;
            var group = new[] { i }.Concat(descriptor.MapPoints[i].Neighbors).Distinct()
                .Where(n => n >= 0 && n < descriptor.MapPoints.Count && descriptor.MapPoints[n].RoadSize == RoadSize.Polygon).ToArray();
            if (group.Length < 3) continue;
            var positions = group.Select(n => Geometry.Enu(transform.Apply(descriptor.MapPoints[n].Position))).ToArray();
            var centre = positions.Aggregate(Vector3.Zero, (sum, p) => sum + p) / positions.Length;
            var ordered = positions.OrderBy(p => Math.Atan2(p.Y - centre.Y, p.X - centre.X)).ToArray();
            for (var n = 0; n < ordered.Length; n++) polygons.Add(new($"{prefab.Uid}:display:{i}", prefab.Uid, n,
                ordered[n].X, ordered[n].Y, ordered[n].Z, "ui_map_display_only"));
            visited.UnionWith(group);
        }
        if (descriptor.Nodes.Count >= 3 && descriptor.NavCurves.Count >= 6)
        {
            var score = unchecked(prefab.Uid * 11400714819323198485UL);
            var worst = junctionSamples.Count > 0 ? junctionSamples.Keys.MaxBy(uid => unchecked(uid * 11400714819323198485UL)) : 0;
            if (junctionSamples.Count < 20 || score < unchecked(worst * 11400714819323198485UL))
            {
                if (junctionSamples.Count == 20) junctionSamples.Remove(worst);
                var sourcePosition = prefab.Nodes[0].Position;
                var gaps = descriptor.NavCurves.SelectMany((curve, index) => curve.NextLines.Where(curvePoints.ContainsKey)
                    .Select(next => (double)Vector3.Distance(curvePoints[index][^1], curvePoints[next][0]))).ToArray();
                junctionSamples[prefab.Uid] = new
                {
                    prefab_uid = prefab.Uid.ToString(), model = prefab.Model.String, origin = prefab.Origin,
                    scs_position = new { x = sourcePosition.X, y = sourcePosition.Y, z = sourcePosition.Z },
                    sector = new { x = (int)Math.Floor(sourcePosition.X / 4000), z = (int)Math.Floor(sourcePosition.Z / 4000) },
                    control_node_max_residual_m = transform.MaxResidual, nav_curve_count = descriptor.NavCurves.Count,
                    native_link_count = gaps.Length, maximum_native_link_gap_m = gaps.Length > 0 ? gaps.Max() : 0,
                    semaphore_count = descriptor.Semaphores.Count,
                    numerical_join_check_passed = gaps.All(gap => gap <= 0.05),
                    game_visual_check_passed = (bool?)null,
                    lanes = curvePoints.Select(pair => new
                    {
                        id = $"{prefab.Uid}:prefab:{pair.Key}",
                        center = pair.Value.Select(p => new { x = p.X, y = p.Y, z = p.Z }).ToArray(),
                        successors = descriptor.NavCurves[pair.Key].NextLines.Select(next => $"{prefab.Uid}:prefab:{next}").ToArray(),
                    }).ToArray(),
                };
            }
        }
    }

    /// <summary>Compare generated road centres with native prefab boundary lane centres, without claiming lane-edge widths.</summary>
    private async Task WriteRoadAudit(Map map, string directory)
    {
        var prefabIds = map.MapItems.Values.OfType<Prefab>().Select(p => p.Uid).ToHashSet();
        var incoming = ends.Where(e => prefabIds.Contains(e.Owner)).ToLookup(e => e.Node);
        var outgoing = starts.Where(e => prefabIds.Contains(e.Owner)).ToLookup(e => e.Node);
        var observations = new List<RoadOffsetObservation>();
        foreach (var road in map.MapItems.Values.OfType<Road>())
        {
            if (!definitions.RoadLooks.TryGetValue(road.RoadType.String, out var look)) continue;
            foreach (var side in new[] { "left", "right" })
            {
                var laneRules = definitions.ResolveLanes(look, side, side == "left" ? road.Left : road.Right);
                if (laneRules.Length == 0) continue;
                var reverse = (side == "left") != road.LeftHandTraffic;
                var offsets = Definitions.Array(look, "lane_offsets_" + side);
                const float width = 4.5f;
                foreach (var (node, atStart) in new[] { (road.Node, true), (road.ForwardNode, false) })
                {
                    var point = Geometry.Enu(node.Position);
                    var right = Geometry.Enu(Vector3.Transform(Vector3.UnitX, node.Rotation));
                    var forward = Geometry.Enu(Vector3.Transform(-Vector3.UnitZ, node.Rotation)) * (reverse ? -1 : 1);
                    var candidates = (atStart != reverse ? incoming[node.Uid] : outgoing[node.Uid])
                        .Where(e => Vector3.Dot(e.Direction, forward) > 0.8)
                        .Select(e => Vector3.Dot(e.Position - point, right) * (side == "left" ? -1 : 1))
                        // One-way roads may place all their lanes left of the
                        // road line (L1: three lanes at -6.75..+2.25 m, the
                        // -4.5 m cut dropped one and mislinked the rest);
                        // opposing lanes are already excluded by heading.
                        .Where(offset => offset > -4.5 * laneRules.Length - 1.0 && offset < 40)
                        .GroupBy(offset => Math.Round(offset / 0.05)).Select(group => group.Average()).Order().ToArray();
                    if (candidates.Length != laneRules.Length) continue;
                    for (var i = 0; i < candidates.Length; i++)
                    {
                        var correction = i < offsets.Length && offsets[i] is Vector2 v ? v.X : 0;
                        var estimate = Definitions.Number(look, "road_offset") + width * (i + 0.5f) + correction;
                        observations.Add(new(road.RoadType.String, (side == "left" ? road.Left : road.Right).Variant.String,
                            side, i, candidates[i], estimate, road.Uid, node.Uid));
                    }
                }
            }
        }
        // Hold out entire road items, including both endpoints, to avoid reporting only fit consistency.
        var training = observations.Where(o => unchecked(o.road_uid * 11400714819323198485UL) % 5 != 0).ToArray();
        var holdout = observations.Where(o => unchecked(o.road_uid * 11400714819323198485UL) % 5 == 0).ToArray();
        var groups = training.GroupBy(o => (o.road_look, o.variant, o.side, o.lane_index)).Select(group =>
        {
            var measured = group.Select(o => o.measured_offset_m).Order().ToArray();
            var estimate = group.First().estimated_offset_m;
            var median = measured[measured.Length / 2];
            var deviations = measured.Select(m => Math.Abs(m - median)).Order().ToArray();
            return new
            {
                group.Key.road_look, group.Key.variant, group.Key.side, group.Key.lane_index,
                samples = measured.Length, median_offset_m = median, estimated_offset_m = estimate,
                median_error_m = median - estimate, p95_deviation_m = deviations[(int)Math.Floor((deviations.Length - 1) * 0.95)],
                lane_edge_width_verified = false,
            };
        }).OrderBy(g => g.road_look).ThenBy(g => g.variant).ThenBy(g => g.side).ThenBy(g => g.lane_index).ToArray();
        foreach (var group in groups.Where(g => g.samples >= 5 && g.p95_deviation_m <= 0.05))
            roadCalibration[(group.road_look, group.variant, group.side, group.lane_index)] = group.median_offset_m;
        // A pooled fallback is accepted only when all observed variants form a tight distribution.
        foreach (var group in training.GroupBy(o => (o.road_look, o.side, o.lane_index)))
        {
            var values = group.Select(o => o.measured_offset_m).Order().ToArray();
            var median = values[values.Length / 2];
            var deviation = values.Select(v => Math.Abs(v - median)).Order().ToArray();
            if (values.Length >= 10 && deviation[(int)((deviation.Length - 1) * 0.95)] <= 0.03)
                roadCalibration[(group.Key.road_look, "*", group.Key.side, group.Key.lane_index)] = median;
        }
        // A road item whose look has no statistical calibration is still
        // located exactly by the native prefab lanes at its own ends (both
        // ends must agree within 0.2 m). Without this, its nominal centres
        // missed the prefab lanes by over 2 m, no link was built and every
        // navigation route through it broke (Kiruna, r11).
        foreach (var group in observations.GroupBy(o => (o.road_uid, o.side, o.lane_index)))
        {
            var values = group.Select(o => o.measured_offset_m).ToArray();
            if (values.Max() - values.Min() <= 0.2)
                roadInstanceCalibration[group.Key] = values.Average();
        }
        // Road items joined end to end (no prefab between them) share the
        // offsets of a calibrated neighbour of the same look.
        var allRoads = map.MapItems.Values.OfType<Road>().ToDictionary(r => r.Uid);
        var roadsAtNode = allRoads.Values
            .SelectMany(r => new[] { (Node: r.Node.Uid, Road: r), (Node: r.ForwardNode.Uid, Road: r) })
            .ToLookup(p => p.Node, p => p.Road);
        var pending = new Queue<ulong>(roadInstanceCalibration.Keys.Select(k => k.Road).Distinct());
        while (pending.TryDequeue(out var uid))
        {
            var road = allRoads[uid];
            foreach (var node in new[] { road.Node.Uid, road.ForwardNode.Uid })
                foreach (var other in roadsAtNode[node])
                {
                    if (other.Uid == uid || other.RoadType.String != road.RoadType.String) continue;
                    // Head to tail keeps the travel direction; head to head swaps sides.
                    var sameDirection = (node == road.Node.Uid) != (node == other.Node.Uid);
                    var added = false;
                    foreach (var side in new[] { "left", "right" })
                    {
                        var otherSide = sameDirection ? side : side == "left" ? "right" : "left";
                        for (var i = 0; i < 16; i++)
                        {
                            if (!roadInstanceCalibration.TryGetValue((uid, side, i), out var offset)
                                || roadInstanceCalibration.ContainsKey((other.Uid, otherSide, i))) continue;
                            roadInstanceCalibration[(other.Uid, otherSide, i)] = offset;
                            added = true;
                        }
                    }
                    if (added) pending.Enqueue(other.Uid);
                }
        }
        Validation["road_lane_instance_offsets"] = roadInstanceCalibration.Count;
        var errors = observations.Select(o => Math.Abs(o.measured_offset_m - o.estimated_offset_m)).Order().ToArray();
        var p95 = errors.Length > 0 ? errors[(int)Math.Floor((errors.Length - 1) * 0.95)] : 0;
        var calibratedErrors = observations.Select(o => Math.Abs(o.measured_offset_m -
            (roadCalibration.TryGetValue((o.road_look, o.variant, o.side, o.lane_index), out var calibrated)
            || roadCalibration.TryGetValue((o.road_look, "*", o.side, o.lane_index), out calibrated) ? calibrated : o.estimated_offset_m))).Order().ToArray();
        var calibratedP95 = calibratedErrors.Length > 0 ? calibratedErrors[(int)Math.Floor((calibratedErrors.Length - 1) * 0.95)] : 0;
        var heldoutErrors = holdout.Select(o => Math.Abs(o.measured_offset_m -
            (roadCalibration.TryGetValue((o.road_look, o.variant, o.side, o.lane_index), out var calibrated)
            || roadCalibration.TryGetValue((o.road_look, "*", o.side, o.lane_index), out calibrated) ? calibrated : o.estimated_offset_m))).Order().ToArray();
        var heldoutP95 = heldoutErrors.Length > 0 ? heldoutErrors[(int)Math.Floor((heldoutErrors.Length - 1) * 0.95)] : 0;
        Validation["road_prefab_endpoint_observations"] = observations.Count;
        Validation["road_prefab_endpoint_offset_error_before_calibration_p95_m"] = p95;
        Validation["road_prefab_endpoint_offset_error_after_calibration_p95_m"] = calibratedP95;
        Validation["road_offset_holdout_observations"] = holdout.Length;
        Validation["road_offset_holdout_error_p95_m"] = heldoutP95;
        await File.WriteAllTextAsync(Path.Combine(directory, "road-offset-audit.json"), JsonSerializer.Serialize(new
        {
            method = "Native prefab boundary centres matched by shared node UID, travel heading, side, and exact lane count",
            observations = observations.Count, offset_error_p95_m = p95,
            calibrated_offset_error_p95_m = calibratedP95,
            training_observations = training.Length, holdout_observations = holdout.Length, holdout_error_p95_m = heldoutP95,
            split_rule = "unsigned overflow (road_uid * 11400714819323198485) modulo 5 == 0 selects held-out road items",
            calibration_rule = "variant group: >=5 observations and P95 deviation <=0.05 m; pooled variants: >=10 and <=0.03 m",
            note = "Centre offsets and adjacent-centre spacing do not determine physical lane-edge widths. No certification is inferred.",
            groups,
        }, Program.Json));
    }

    private async Task AddLane(string id, ulong owner, ulong nodeStart, ulong nodeEnd, string kind, float? width,
        string rule, string status, List<Vector3> points, PointWriter writer, LanePolicy policy, bool centerlineValidated)
    {
        var start = points[0];
        lanes.Add(new(id, owner, nodeStart, nodeEnd, kind, width, null, rule, status,
            (int)Math.Floor(start.X / 4000), (int)Math.Floor(-start.Y / 4000),
            points.Min(p => p.X), points.Min(p => p.Y), points.Min(p => p.Z),
            points.Max(p => p.X), points.Max(p => p.Y), points.Max(p => p.Z), kind == "road" ? 4.5f : null,
            centerlineValidated, policy.TruckAccess, policy.SpeedClass, "runtime_telemetry_required"));
        if (nodeStart != 0) starts.Add(new(id, owner, nodeStart, points[0], Geometry.Direction(points, true)));
        if (nodeEnd != 0) ends.Add(new(id, owner, nodeEnd, points[^1], Geometry.Direction(points, false)));
        await writer.Add(id, points);
        if (runtimeGeometry is not null)
            await runtimeGeometry.WriteLineAsync(JsonSerializer.Serialize(new
            {
                id, owner_uid = owner, node_start = nodeStart, node_end = nodeEnd,
                center = points.Select(p => new { x = (double)p.X, y = (double)p.Y, z = (double)p.Z }),
                width = (double?)null, speed_limit = (double?)null, geometry_status = status,
                navigation_width = kind == "road" ? (float?)4.5f : null, centerline_validated = centerlineValidated,
                physical_boundaries_verified = false, truck_access = policy.TruckAccess, speed_class = policy.SpeedClass,
                speed_source = "runtime_telemetry_required",
            }));
    }

    private async Task WriteRuntime(string directory)
    {
        var successors = links.ToLookup(l => l.lane_id, l => l.successor_id);
        var predecessors = links.ToLookup(l => l.successor_id, l => l.lane_id);
        await using var output = File.Create(Path.Combine(directory, "runtime-map.json"));
        await using var json = new Utf8JsonWriter(output);
        json.WriteStartObject();
        json.WriteNumber("schema_version", 1);
        json.WriteBoolean("accepted_for_autonomous_driving", false);
        json.WriteString("source", "installed_game_archives");
        json.WriteStartArray("lanes");
        using var input = File.OpenText(Path.Combine(directory, "runtime-geometry.jsonl"));
        while (await input.ReadLineAsync() is { } line)
        {
            using var document = JsonDocument.Parse(line);
            var id = document.RootElement.GetProperty("id").GetString()!;
            json.WriteStartObject();
            foreach (var property in document.RootElement.EnumerateObject()) property.WriteTo(json);
            json.WritePropertyName("successors"); JsonSerializer.Serialize(json, successors[id].Distinct());
            json.WritePropertyName("predecessors"); JsonSerializer.Serialize(json, predecessors[id].Distinct());
            json.WriteEndObject();
            await json.FlushAsync();
        }
        json.WriteEndArray();
        json.WritePropertyName("conflicts"); JsonSerializer.Serialize(json, conflictPairs);
        json.WritePropertyName("lane_controls"); JsonSerializer.Serialize(json, laneControls);
        json.WritePropertyName("signals"); JsonSerializer.Serialize(json, signals);
        json.WritePropertyName("signal_lanes"); JsonSerializer.Serialize(json, signalLanes);
        json.WritePropertyName("stop_lines"); JsonSerializer.Serialize(json, stops);
        json.WriteEndObject();
    }

    private void BuildLinks()
    {
        var byNode = starts.ToLookup(e => e.Node);
        foreach (var end in ends)
        {
            var candidates = byNode[end.Node].Where(start => start.Owner != end.Owner && Vector3.Dot(end.Direction, start.Direction) > 0.5)
                .Select(start => (Start: start, Gap: Vector3.Distance(end.Position, start.Position)))
                .Where(pair => pair.Gap < 2).OrderBy(pair => pair.Gap).ToArray();
            if (candidates.Length > 0)
                links.Add(new(end.LaneId, candidates[0].Start.LaneId, "shared_native_node_and_geometry", candidates[0].Gap));
        }
    }

    private async Task WriteSlotAudit(string directory)
    {
        var ppdSlots = slots.Where(s => s.node_uid == 0).ToLookup(s => s.owner_uid);
        var rows = new List<object>();
        var positionErrors = new List<double>();
        var yawErrors = new List<double>();
        var unmatched = 0;
        foreach (var (slot, prefab) in companySlots)
        {
            var candidates = ppdSlots[prefab].Select(other => (Other: other,
                Distance: Math.Sqrt(Math.Pow(slot.x - other.x, 2) + Math.Pow(slot.y - other.y, 2) + Math.Pow(slot.z - other.z, 2))))
                .OrderBy(pair => pair.Distance).ToArray();
            if (candidates.Length == 0) { unmatched++; continue; }
            var best = candidates[0];
            var yaw = Math.Abs(Math.IEEERemainder(slot.yaw - best.Other.yaw, Math.Tau));
            positionErrors.Add(best.Distance);
            if (best.Distance <= 0.05) yawErrors.Add(yaw);
            rows.Add(new { native_node_uid = slot.node_uid.ToString(), prefab_uid = prefab.ToString(), slot.company, slot.city,
                native_kind = slot.kind, ppd_kind = best.Other.kind, position_error_m = best.Distance, yaw_error_rad = yaw });
        }
        static double Percentile(List<double> values) => values.Count == 0 ? 0 : values.Order().ElementAt((int)((values.Count - 1) * 0.95));
        Validation["company_slot_ppd_position_error_p95_m"] = Percentile(positionErrors);
        Validation["company_slot_ppd_yaw_error_p95_rad"] = Percentile(yawErrors);
        Validation["company_slot_unmatched"] = unmatched;
        await File.WriteAllTextAsync(Path.Combine(directory, "slot-audit.json"), JsonSerializer.Serialize(new
        {
            method = "Native Company spawn nodes compared to nearest PPD spawn on the same parent prefab; yaw evaluated only within 0.05 m",
            native_slots = companySlots.Count, compared_slots = rows.Count, unmatched,
            position_error_p95_m = Percentile(positionErrors), yaw_error_p95_rad = Percentile(yawErrors),
            rows,
        }, Program.Json));
    }

    private async Task Write<T>(string name, IEnumerable<T> rows, string directory)
    {
        var data = rows.ToArray();
        await ParquetSerializer.SerializeAsync(data, Path.Combine(directory, name + ".parquet"));
        Counts[name] = data.LongLength;
    }
}

internal sealed class PointWriter : IAsyncDisposable
{
    private static readonly DataField<string> Lane = new("lane_id");
    private static readonly DataField<int> Index = new("point_index");
    private static readonly DataField<double> X = new("x");
    private static readonly DataField<double> Y = new("y");
    private static readonly DataField<double> Z = new("z");
    private readonly Stream stream;
    private readonly ParquetWriter writer;
    private readonly List<(string Id, int Index, Vector3 Point)> pending = new(65536);
    public long Count { get; private set; }
    private PointWriter(Stream stream, ParquetWriter writer) { this.stream = stream; this.writer = writer; }
    public static async Task<PointWriter> Create(string path)
    {
        var stream = File.Create(path);
        try { return new(stream, await ParquetWriter.CreateAsync(new ParquetSchema(Lane, Index, X, Y, Z), stream)); }
        catch { await stream.DisposeAsync(); throw; }
    }
    public async Task Add(string id, List<Vector3> points)
    {
        for (var i = 0; i < points.Count; i++)
        {
            pending.Add((id, i, points[i]));
            if (pending.Count >= 65536) await Flush();
        }
    }
    public async Task Flush()
    {
        if (pending.Count == 0) return;
        using var group = writer.CreateRowGroup();
        await group.WriteAsync(Lane, pending.Select(p => p.Id).ToArray());
        await group.WriteAsync<int>(Index, pending.Select(p => p.Index).ToArray());
        await group.WriteAsync<double>(X, pending.Select(p => (double)p.Point.X).ToArray());
        await group.WriteAsync<double>(Y, pending.Select(p => (double)p.Point.Y).ToArray());
        await group.WriteAsync<double>(Z, pending.Select(p => (double)p.Point.Z).ToArray());
        Count += pending.Count;
        pending.Clear();
    }
    public async ValueTask DisposeAsync() { await Flush(); await writer.DisposeAsync(); await stream.DisposeAsync(); }
}
