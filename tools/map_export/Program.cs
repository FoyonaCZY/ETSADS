// SPDX-License-Identifier: GPL-2.0-only
using System.Diagnostics;
using System.Globalization;
using System.Security.Cryptography;
using System.Text;
using System.Text.Json;
using TruckLib;
using TruckLib.HashFs;
using TruckLib.ScsMap;
using TruckLib.Sii;
using Parquet;

namespace Etsads.MapExport;

internal static class Program
{
    internal static readonly JsonSerializerOptions Json = new() { WriteIndented = true, IncludeFields = true };

    private static async Task<int> Main(string[] args)
    {
        try
        {
            if (args.Length == 0 || args.Contains("--help"))
            {
                Console.WriteLine("""
                ETSADS offline map exporter (GPL-2.0-only, .NET 10)
                inspect --game-dir PATH [--output PATH] [--mod PATH ...]
                export  --game-dir PATH [--output PATH] [--mod PATH ...] [--sector X,Z ...] [--runtime-json]
                self-test
                verify --input EXPORT_DIRECTORY
                slice --input EXPORT_DIRECTORY --x ENU_X --y ENU_Y --radius METRES --output DIRECTORY
                partition --input PARQUET_EXPORT --output NEW_DIRECTORY
                partition-metadata --input PARQUET_EXPORT --output EXISTING_SECTOR_DIRECTORY [--game-dir PATH] [--mod PATH ...]
                inspect-signal-profiles --game-dir PATH --output NEW_DIRECTORY
                inspect-save --input info.sii
                Mods are explicit, highest priority first. Original game files are read-only.
                Outputs are ENU metres. Unverified geometry is marked in manifest.json.
                """);
                return 0;
            }
            if (args[0] == "self-test") { SignalProfileResolver.SelfTest(); Console.WriteLine("Passed: signal profile inheritance, country/prefab overrides, ID fallback, tuple ordering, malformed/nonfinite timing rejection."); return await GeometryTests.Run(); }
            if (args[0] == "slice") return await MapSlice.Run(args);
            if (args[0] == "partition") return await Partition.Run(args);
            if (args[0] == "partition-metadata") return await AnnotationPartition.Run(args);
            if (args[0] == "inspect-signal-profiles") return await SignalProfiles.Inspect(args);
            if (args[0] == "inspect-save") return SaveInspect.Run(args);
            if (args[0] == "verify")
            {
                if (args.Length != 3 || args[1] != "--input") throw new ArgumentException("Expected verify --input EXPORT_DIRECTORY.");
                return await Verify(Path.GetFullPath(args[2]));
            }
            var options = Options.Parse(args);
            var timer = Stopwatch.StartNew();
            using var archives = ArchiveSet.Open(options);
            var output = Path.Combine(options.Output, archives.CacheKey);
            Directory.CreateDirectory(output);
            await File.WriteAllTextAsync(Path.Combine(output, "archives.json"),
                JsonSerializer.Serialize(archives.Inventory, Json));
            var definitions = Definitions.Load(archives.FileSystem);
            await File.WriteAllTextAsync(Path.Combine(output, "definition_samples.json"),
                JsonSerializer.Serialize(definitions.Samples, Json));
            if (options.Command == "inspect")
            {
                await File.WriteAllTextAsync(Path.Combine(output, "definition-catalog.json"), JsonSerializer.Serialize(new
                {
                    road_looks = definitions.RoadLooks.Values.Select(Definitions.Inspect),
                    traffic_lanes = definitions.TrafficLanes.Values.Select(Definitions.Inspect),
                    road_variants = definitions.RoadVariants.Values.Select(Definitions.Inspect),
                    traffic_rules = definitions.TrafficRules.Values.Select(Definitions.Inspect),
                }, Json));
                Console.WriteLine(JsonSerializer.Serialize(new { archives.Version, archives.CacheKey,
                    Archives = archives.Inventory.Count, definitions.RoadLooks.Count,
                    PrefabDefinitions = definitions.Prefabs.Count, Output = output }, Json));
                return 0;
            }
            Console.WriteLine($"Reading map (ETS2 {archives.Version}), {archives.Inventory.Count} archives...");
            var map = Map.Open("/map/europe.mbd", archives.FileSystem,
                options.Sectors.Count == 0 ? null : options.Sectors);
            Console.WriteLine($"Read {map.Sectors.Count} sectors, {map.MapItems.Count} items, {map.Nodes.Count} nodes.");
            File.Delete(Path.Combine(output, "manifest.json")); // No stale success marker if this run fails.
            var exporter = new Exporter(archives.FileSystem, definitions);
            exporter.ProfileResolver = SignalProfileResolver.Load(archives);
            exporter.GameVersion = archives.Version;
            exporter.WriteRuntimeJson = options.RuntimeJson;
            exporter.BaseMapSectors = archives.BaseMapSectors;
            await exporter.Export(map, output);
            var signalProfileEvidence=await File.ReadAllBytesAsync(Path.Combine(output,"signal-profile-evidence.json"));
            var manifest = new
            {
                schema_version = 1, game_version = archives.Version, cache_key = archives.CacheKey,
                game_version_source = "bin/win_x64/eurotrucks2.exe ProductVersion",
                game_file_version = archives.FileVersion,
                exporter_sha256 = archives.ExporterHash,
                coordinate_system = "ENU", units = "metres", source = "installed_game_archives",
                map_format = 907, trucklib = "0.5.1", parquet_net = "6.1.0",
                generated_utc = DateTimeOffset.UtcNow, elapsed_seconds = timer.Elapsed.TotalSeconds,
                full_map = options.Sectors.Count == 0, sectors = map.Sectors.Count,
                archives = archives.Inventory.Select(x => x.Name),
                counts = exporter.Counts, warnings = exporter.Warnings,
                validation = exporter.Validation,
                signal_profile_resolution = new { evidence_path="signal-profile-evidence.json", evidence_bytes=signalProfileEvidence.Length,
                    evidence_sha256=Convert.ToHexStringLower(SHA256.HashData(signalProfileEvidence)), activation_verified=false },
                accepted_for_autonomous_driving = false,
                capabilities = new
                {
                    interpretation = "The legacy acceptance flag reports full-map game acceptance, not a prohibition on validated regional use.",
                    native_nodes_and_company_slots = "available",
                    centerline_topology = "native prefab curves plus per-lane calibrated/estimated road status",
                    nominal_road_navigation_width_m = 4.5,
                    nominal_width_source = "https://modding.scssoft.com/wiki/Games/ETS2/Modding_guides/1.15",
                    physical_boundaries = "requires regional geometry or game projection evidence",
                    speed_limits = "map unknown; runtime may use fresh truck.navigation.speed.limit with fail-closed freshness checks",
                    regional_driving = "requires lane-specific centerline/width evidence and runtime safety validation",
                },
                pending_acceptance = new[] { "20 junction connectivity visual checks", "in-game lane overlay alignment", "regional runtime speed-limit freshness and safety checks", "authoritative drivable surface polygons" },
            };
            var manifestText = JsonSerializer.Serialize(manifest, Json);
            await File.WriteAllTextAsync(Path.Combine(output, "manifest.json"), manifestText);
            var candidateDirectory = Path.Combine(output, "highway-candidates");
            if (Directory.Exists(candidateDirectory))
                await File.WriteAllTextAsync(Path.Combine(candidateDirectory, "manifest.json"), JsonSerializer.Serialize(new
                {
                    schema_version = 1, source = "installed_game_archives", source_cache_key = archives.CacheKey,
                    game_version = archives.Version, game_file_version = archives.FileVersion,
                    game_version_source = "bin/win_x64/eurotrucks2.exe ProductVersion",
                    source_manifest_sha256 = Convert.ToHexStringLower(SHA256.HashData(Encoding.UTF8.GetBytes(manifestText))),
                    accepted_for_autonomous_driving = false,
                    scope = "Bounded candidate runtime JSON files derived from the parent export; physical/in-game acceptance pending.",
                }, Json));
            Console.WriteLine(JsonSerializer.Serialize(manifest, Json));
            Console.WriteLine($"Output: {output}");
            return 0;
        }
        catch (Exception ex)
        {
            Console.Error.WriteLine($"Map export failed: {ex.Message}");
            if (Environment.GetEnvironmentVariable("ETSADS_DEBUG") == "1") Console.Error.WriteLine(ex);
            return 1;
        }
    }

    private static async Task<int> Verify(string directory)
    {
        using var manifest = JsonDocument.Parse(await File.ReadAllTextAsync(Path.Combine(directory, "manifest.json")));
        var counts = manifest.RootElement.GetProperty("counts");
        var results = new List<object>();
        foreach (var file in Directory.GetFiles(directory, "*.parquet").Order(StringComparer.Ordinal))
        {
            var name = Path.GetFileNameWithoutExtension(file);
            if (!counts.TryGetProperty(name, out var expected)) throw new InvalidDataException($"Unlisted table {name}.");
            await using var reader = await ParquetReader.CreateAsync(file);
            long rows = 0;
            for (var groupIndex = 0; groupIndex < reader.RowGroupCount; groupIndex++)
            {
                using var group = reader.OpenRowGroupReader(groupIndex);
                rows += group.RowCount;
                // Decode every field, checking codecs/pages rather than trusting footer counts only.
                foreach (var field in reader.Schema.GetDataFields())
                    using (await group.ReadRawColumnDataBaseAsync(field)) { }
            }
            if (rows != expected.GetInt64()) throw new InvalidDataException($"{name}: expected {expected}, decoded {rows} rows.");
            results.Add(new { table = name, rows, row_groups = reader.RowGroupCount, columns = reader.Schema.GetDataFields().Select(f => f.Name) });
            Console.WriteLine($"Verified {name}: {rows:N0} rows.");
        }
        await File.WriteAllTextAsync(Path.Combine(directory, "verification.json"), JsonSerializer.Serialize(new
        {
            verified_utc = DateTimeOffset.UtcNow, all_parquet_pages_decoded = true,
            scope = "file integrity and row counts; no in-game acceptance implied", tables = results,
        }, Json));
        return 0;
    }
}

internal sealed record Options(string Command, string GameDirectory, string Output, List<string> Mods, List<SectorCoordinate> Sectors, bool RuntimeJson)
{
    public static Options Parse(string[] args)
    {
        if (args[0] is not ("inspect" or "export")) throw new ArgumentException("Expected inspect, export, or self-test. Use --help.");
        string? game = null;
        var output = Path.GetFullPath("tools/map_export/output");
        var mods = new List<string>();
        var sectors = new List<SectorCoordinate>();
        var runtimeJson = false;
        for (var i = 1; i < args.Length; i++)
        {
            if (args[i] == "--runtime-json") { runtimeJson = true; continue; }
            if (i + 1 == args.Length) throw new ArgumentException($"Missing value for {args[i]}.");
            var key = args[i++];
            var value = args[i];
            switch (key)
            {
                case "--game-dir": game = Path.GetFullPath(value); break;
                case "--output": output = Path.GetFullPath(value); break;
                case "--mod": mods.Add(Path.GetFullPath(value)); break;
                case "--sector":
                    var parts = value.Split(',');
                    if (parts.Length != 2 || !int.TryParse(parts[0], out var x) || !int.TryParse(parts[1], out var z))
                        throw new ArgumentException("Sector must be X,Z (integer coordinates).");
                    sectors.Add(new SectorCoordinate(x, z));
                    break;
                default: throw new ArgumentException($"Unknown option {key}.");
            }
        }
        if (game is null || !File.Exists(Path.Combine(game, "base_map.scs")))
            throw new ArgumentException("--game-dir must point to an installed ETS2 directory containing base_map.scs.");
        return new(args[0], game, output, mods, sectors, runtimeJson);
    }
}

internal sealed record ArchiveInfo(string Name, long Bytes, DateTime ModifiedUtc, int MapBaseSectors, string[] Maps);

internal sealed class ArchiveSet : IDisposable
{
    private readonly List<IHashFsReader> readers = [];
    private readonly List<string> archivePaths = [];
    private readonly Dictionary<string,string> archiveHashes = [];
    public required AssetLoader FileSystem { get; init; }
    public required List<ArchiveInfo> Inventory { get; init; }
    public required string Version { get; init; }
    public string? FileVersion { get; init; }
    public required string CacheKey { get; init; }
    public required string ExporterHash { get; init; }
    public HashSet<SectorCoordinate> BaseMapSectors { get; } = [];

    public static ArchiveSet Open(Options options)
    {
        var exe = Path.Combine(options.GameDirectory, "bin", "win_x64", "eurotrucks2.exe");
        var executableVersion = File.Exists(exe) ? FileVersionInfo.GetVersionInfo(exe) : null;
        var version = executableVersion?.ProductVersion ?? "unknown";
        if (!version.StartsWith("1.59.") && !version.StartsWith("1.60.") && !version.StartsWith("1.61."))
            throw new NotSupportedException($"Game {version} is outside TruckLib's verified 1.59–1.61 range.");
        // Explicit mods highest first. DLC overrides base. Exact order recorded in the cache identity.
        var paths = options.Mods.Concat(Directory.GetFiles(options.GameDirectory, "dlc_*.scs").OrderDescending(StringComparer.Ordinal))
            .Concat(new[] { "def.scs", "base_map.scs", "base.scs", "base_share.scs", "base_cfg.scs", "core.scs" }
                .Select(name => Path.Combine(options.GameDirectory, name)).Where(File.Exists)).Distinct().ToArray();
        var readers = new List<IHashFsReader>();
        var inventory = new List<ArchiveInfo>();
        var baseMapSectors = new HashSet<SectorCoordinate>();
        try
        {
            foreach (var path in paths)
            {
                var reader = HashFsReader.Open(path);
                readers.Add(reader);
                var mapFiles = reader.EntryExists("/map/") == EntryType.Directory
                    ? reader.GetDirectoryListing("/map/").Files.ToArray() : [];
                var sectors = reader.EntryExists("/map/europe/") == EntryType.Directory
                    ? reader.GetDirectoryListing("/map/europe/").Files.Count(p => p.EndsWith(".base", StringComparison.Ordinal)) : 0;
                if (path == Path.Combine(options.GameDirectory, "base_map.scs"))
                    baseMapSectors.UnionWith(reader.GetDirectoryListing("/map/europe/").Files.Where(p => p.EndsWith(".base", StringComparison.Ordinal)).Select(Sector.SectorCoordsFromSectorFilePath));
                var info = new FileInfo(path);
                inventory.Add(new(info.Name, info.Length, info.LastWriteTimeUtc, sectors, mapFiles));
            }
            var exporterHash = Convert.ToHexStringLower(SHA256.HashData(File.ReadAllBytes(typeof(Program).Assembly.Location)));
            var identity = "schema=1\nexporter=" + exporterHash + "\n" + version + "\n" + string.Join('\n', paths.Select((p, i) => $"{p}|{inventory[i].Bytes}|{inventory[i].ModifiedUtc.Ticks}"))
                + "\nsectors=" + string.Join(';', options.Sectors);
            var result = new ArchiveSet { FileSystem = new AssetLoader(readers.Cast<IFileSystem>().ToArray()), Inventory = inventory,
                Version = version, FileVersion = executableVersion?.FileVersion,
                CacheKey = Convert.ToHexStringLower(SHA256.HashData(Encoding.UTF8.GetBytes(identity)))[..12], ExporterHash = exporterHash };
            result.readers.AddRange(readers);
            result.archivePaths.AddRange(paths);
            result.BaseMapSectors.UnionWith(baseMapSectors);
            return result;
        }
        catch { foreach (var reader in readers) reader.Dispose(); throw; }
    }
    public void Dispose() { foreach (var reader in readers) reader.Dispose(); }
    public object Origin(string path)
    {
        for(var i=0;i<readers.Count;i++) if (((IFileSystem)readers[i]).FileExists(path))
        {
            var archive=archivePaths[i];
            if(!archiveHashes.TryGetValue(archive,out var hash))
            { using var stream=File.OpenRead(archive); archiveHashes[archive]=hash=Convert.ToHexStringLower(SHA256.HashData(stream)); }
            return new { archive=Inventory[i].Name, archive_sha256=hash, archive_bytes=Inventory[i].Bytes,
                archive_modified_utc=Inventory[i].ModifiedUtc, archive_priority=i, asset_path=path,
                asset_sha256=Convert.ToHexStringLower(SHA256.HashData(FileSystem.ReadAllBytes(path))) };
        }
        throw new FileNotFoundException("No source archive for asset.",path);
    }
}

internal sealed class Definitions
{
    public Dictionary<string, Unit> RoadLooks { get; } = [];
    public Dictionary<string, string> Prefabs { get; } = [];
    public Dictionary<string, Unit> TrafficLanes { get; } = [];
    public Dictionary<string, Unit> RoadVariants { get; } = [];
    public Dictionary<string, Unit> TrafficRules { get; } = [];
    public List<object> Samples { get; } = [];
    public static Definitions Load(IFileSystem fs)
    {
        var result = new Definitions();
        var paths = fs.GetFiles("/def/world").Concat(fs.GetFiles("/def")).Where(p => Path.GetFileName(p).StartsWith("road_look", StringComparison.Ordinal)
            || Path.GetFileName(p).StartsWith("prefab", StringComparison.Ordinal)
            || Path.GetFileName(p).StartsWith("traffic_lane", StringComparison.Ordinal)
            || Path.GetFileName(p).StartsWith("traffic_rules", StringComparison.Ordinal)).Where(p => p.EndsWith(".sii", StringComparison.Ordinal));
        foreach (var path in paths.Order(StringComparer.Ordinal))
        {
            var sii = SiiFile.Open(path, fs);
            foreach (var unit in sii.Units)
            {
                var token = unit.Name[(unit.Name.LastIndexOf('.') + 1)..];
                if (unit.Class == "road_look") result.RoadLooks[token] = unit;
                if (unit.Class == "prefab_model" && unit.Attributes.TryGetValue("prefab_desc", out object? desc)) result.Prefabs[token] = desc.ToString()!;
                if (unit.Class == "traffic_lane_data") result.TrafficLanes[unit.Name] = unit;
                if (unit.Class == "road_template_variant") result.RoadVariants[unit.Name] = unit;
                if (unit.Class == "traffic_rule_data") result.TrafficRules[unit.Name] = unit;
            }
        }
        foreach (var u in result.RoadLooks.Values.Where(u => u.Attributes.Any(a => Text(a.Value).Contains("motorway", StringComparison.OrdinalIgnoreCase))).Take(2)
            .Concat(result.RoadLooks.Values.Take(5)).Concat(result.TrafficLanes.Values.Take(5)))
            result.Samples.Add(new { u.Class, u.Name, Attributes = u.Attributes.ToDictionary(x => x.Key, x => Describe(x.Value)) });
        return result;
    }
    private static object? Describe(object? value) => value is System.Collections.IEnumerable e && value is not string
        ? e.Cast<object>().Select(Describe).ToArray() : value is null ? null : Text(value);
    public static string Text(object value) => value switch
    {
        LinkPointer p => p.Value, OwnerPointer p => p.Value, Token t => t.String,
        System.Collections.IEnumerable e when value is not string => string.Join(';', e.Cast<object>().Select(Text)),
        _ => value.ToString() ?? "",
    };
    public static object Inspect(Unit unit) => new { unit.Class, unit.Name,
        attributes = unit.Attributes.ToDictionary(pair => pair.Key, pair => Describe(pair.Value)) };

    public object[] ResolveLanes(Unit look, string side, RoadSide placedSide)
    {
        var result = Array(look, "lanes_" + side);
        var selectedLaneVariant = placedSide.TrafficRule.String;
        if (string.IsNullOrEmpty(selectedLaneVariant) && look.Attributes.TryGetValue("lane_variant_default", out object? defaultVariant))
            selectedLaneVariant = Text(defaultVariant);
        var bestSpecificity = -1;
        foreach (var reference in Array(look, "lane_variants").Concat(Array(look, "template_variants_" + side)))
        {
            if (!RoadVariants.TryGetValue(Text(reference), out var variant)) continue;
            if (variant.Attributes.TryGetValue("lane_variant", out object? laneVariant) && Text(laneVariant) != selectedLaneVariant) continue;
            var models = Array(variant, "model_variants").Concat(Array(variant, "variant_names")).Select(Text).ToArray();
            if (models.Length > 0 && !models.Contains(placedSide.Variant.String, StringComparer.Ordinal)) continue;
            var specificity = (models.Length > 0 ? 2 : 0) + (variant.Attributes.ContainsKey("lane_variant") ? 1 : 0);
            if (specificity >= bestSpecificity && variant.Attributes.ContainsKey("lanes_" + side))
            { result = Array(variant, "lanes_" + side); bestSpecificity = specificity; }
        }
        return result;
    }

    public LanePolicy LanePolicy(string laneName)
    {
        if (!TrafficLanes.TryGetValue(laneName, out var lane)) return new(null, null, null, "unknown");
        var names = new HashSet<string>(StringComparer.Ordinal);
        var pending = new Queue<string>(Array(lane, "traffic_rules").Select(Text));
        var complete = true;
        while (pending.TryDequeue(out var name))
        {
            if (!names.Add(name)) continue;
            if (!TrafficRules.TryGetValue(name, out var rule)) { complete = false; continue; }
            foreach (var nested in Array(rule, "additional_rules").Select(Text)) pending.Enqueue(nested);
        }
        bool access = false, denied = false;
        foreach (var name in names)
        {
            if (!TrafficRules.TryGetValue(name, out var rule) || !rule.Attributes.TryGetValue("rule", out object? kind) || Text(kind) != "actor_access") continue;
            var numbers = Array(rule, "num_params").Select(Convert.ToSingle).ToArray();
            var actors = Array(rule, "str_params").Select(Text).ToArray();
            if (numbers.Length < 2) { complete = false; continue; }
            var includesTruck = actors.Any(a => a is "road" or "truck" or "truck_all" or "truck_semi" or "truck_heavy");
            if (numbers[0] == 1 && includesTruck) access = true;
            if ((numbers[0] == 0 && includesTruck) || (numbers[1] == 1 && !includesTruck)) denied = true;
        }
        var speedClass = lane.Attributes.TryGetValue("speed_class", out object? speed) ? Text(speed) : "unknown";
        return new(complete ? access && !denied : null,
            complete ? !names.Contains("traffic_rule.no_chng_in") : null,
            complete ? !names.Contains("traffic_rule.no_chng_out") : null, speedClass);
    }
    public static float Number(Unit unit, string name, float fallback = 0) => unit.Attributes.TryGetValue(name, out object? v)
        ? Convert.ToSingle(v, CultureInfo.InvariantCulture) : fallback;
    public static object[] Array(Unit unit, string name) => unit.Attributes.TryGetValue(name, out object? v) && v is System.Collections.IEnumerable e
        ? e.Cast<object>().ToArray() : [];
}

internal sealed record LanePolicy(bool? TruckAccess, bool? ChangeIn, bool? ChangeOut, string SpeedClass);
