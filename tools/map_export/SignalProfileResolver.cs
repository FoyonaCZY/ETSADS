// SPDX-License-Identifier: GPL-2.0-only
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Security.Cryptography;
using System.Text.Json;
using TruckLib;
using TruckLib.ScsMap;
using TruckLib.Sii;

namespace Etsads.MapExport;

internal sealed record CountryProvenance(string? Country, string Status, ulong Component, ulong[] BorderNodes);
internal sealed record ProfileDefinition(Unit Unit, string Root, string[] Sources);
internal sealed record ResolvedAttribute(object[] Values, string Unit, string Root, string[] Sources);

/// <summary>Actual SCS definition semantics, never profile-name heuristics.
/// Official 1.9: country override, ID-indexed arrays with last-value fallback,
/// inherited missing attributes. Official 1.32: prefab-instance override.</summary>
internal sealed class SignalProfileResolver
{
    private readonly Dictionary<string, ProfileDefinition> profiles = new(StringComparer.Ordinal);
    private readonly HashSet<string> ambiguous = new(StringComparer.Ordinal);
    private readonly Dictionary<byte, string> countries = [];
    private readonly Dictionary<string, object> sources = new(StringComparer.Ordinal);
    private readonly Dictionary<string, object> evidence = new(StringComparer.Ordinal);
    private readonly Dictionary<ulong, CountryProvenance> mapCountries = [];
    private readonly Dictionary<ulong, string> overrides = [];
    public object CountrySummary { get; private set; } = new { status = "map_not_loaded" };
    public object Document(string gameVersion) => new { schema_version = 1, game_version = gameVersion,
        accepted_for_autonomous_driving = false, activation_verified = false,
        semantics_sources = new[] { "https://modding.scssoft.com/wiki/Games/ETS2/Modding_guides/1.9", "https://modding.scssoft.com/wiki/Games/ETS2/Modding_guides/1.32" },
        sources, country_summary = CountrySummary, resolutions = evidence };

    public static SignalProfileResolver Load(ArchiveSet archives)
    {
        var result = new SignalProfileResolver(); var fs = archives.FileSystem;
        var paths = fs.GetFiles("/def/world").Where(p => Path.GetFileName(p).StartsWith("semaphore_profile", StringComparison.Ordinal) && p.EndsWith(".sii", StringComparison.Ordinal))
            .Concat(fs.GetFiles("/def").Where(p => Path.GetFileName(p).StartsWith("country", StringComparison.Ordinal) && p.EndsWith(".sii", StringComparison.Ordinal)));
        foreach (var path in paths.Order(StringComparer.Ordinal))
        {
            var sii = SiiFile.Open(path, fs); var assetPaths = sii.Includes.Append(path).Distinct().Order(StringComparer.Ordinal).ToArray();
            foreach (var asset in assetPaths) result.sources[asset] = archives.Origin(asset);
            foreach (var unit in sii.Units)
            {
                if (unit.Class == "tr_semaphore_profile") result.Add(unit, path, assetPaths);
                if (unit.Class == "country_data" && unit.Attributes.TryGetValue("country_id", out var id))
                {
                    var number = Convert.ToInt32(id); if (number is <= 0 or > 255) continue;
                    var country = unit.Name["country.data.".Length..];
                    if (result.countries.TryGetValue((byte)number, out var old) && old != country)
                        throw new InvalidDataException("Conflicting country IDs in actual definitions.");
                    result.countries[(byte)number] = country;
                }
            }
        }
        return result;
    }
    private void Add(Unit unit, string root, string[] assets)
    {
        if (profiles.TryGetValue(unit.Name, out var old) && JsonSerializer.Serialize(Definitions.Inspect(old.Unit)) != JsonSerializer.Serialize(Definitions.Inspect(unit)))
            ambiguous.Add(unit.Name);
        else profiles[unit.Name] = new(unit, root, assets);
    }
    public void BindMap(Map map)
    {
        var items = map.MapItems.Values.Where(i => i is Road or Prefab).OrderBy(i => i.Uid).ToArray();
        var ids = items.Select((item, index) => (item.Uid, index)).ToDictionary(p => p.Uid, p => p.index);
        var parent = Enumerable.Range(0, items.Length).ToArray();
        int Find(int i) { while (parent[i] != i) { parent[i] = parent[parent[i]]; i = parent[i]; } return i; }
        void Join(int a, int b) { a = Find(a); b = Find(b); if (a != b) parent[Math.Max(a, b)] = Math.Min(a, b); }
        foreach (var node in map.Nodes.Values.Where(n => !n.IsCountryBorder))
            if (node.ForwardItem is { } f && node.BackwardItem is { } b && ids.TryGetValue(f.Uid, out var fi) && ids.TryGetValue(b.Uid, out var bi)) Join(fi, bi);
        var seeds = new Dictionary<int, List<(byte Country, ulong Node)>>();
        void Seed(IMapObject? item, byte country, ulong node)
        {
            if (item is null || country == 0 || !ids.TryGetValue(item.Uid, out var i)) return;
            var root = Find(i); if (!seeds.TryGetValue(root, out var list)) seeds[root] = list = []; list.Add((country, node));
        }
        foreach (var node in map.Nodes.Values.Where(n => n.IsCountryBorder))
        { Seed(node.ForwardItem, node.ForwardCountry, node.Uid); Seed(node.BackwardItem, node.BackwardCountry, node.Uid); }
        var byRoot = new Dictionary<int, CountryProvenance>();
        foreach (var prefab in items.OfType<Prefab>())
        {
            var root = Find(ids[prefab.Uid]);
            if (!byRoot.TryGetValue(root, out var provenance))
            {
                var seed = seeds.GetValueOrDefault(root, []); var values = seed.Select(s => s.Country).Distinct().ToArray();
                var country = values.Length == 1 ? countries.GetValueOrDefault(values[0]) : null;
                provenance = new(country, country is not null ? "native_border_component" : values.Length > 1 ? "conflicting_border_component" : "unseeded_country",
                    items[root].Uid, seed.Select(s => s.Node).Distinct().Order().ToArray());
                byRoot[root] = provenance;
            }
            mapCountries[prefab.Uid] = provenance; overrides[prefab.Uid] = prefab.SemaphoreProfile.String;
        }
        CountrySummary = new { prefab_count = mapCountries.Count, components = byRoot.Count,
            statuses = mapCountries.Values.GroupBy(v => v.Status).ToDictionary(g => g.Key, g => g.Count()),
            countries = mapCountries.Values.Where(v => v.Country is not null).GroupBy(v => v.Country!).ToDictionary(g => g.Key, g => g.Count()) };
    }
    private ResolvedAttribute? Attribute(string unitName, string attribute, HashSet<string>? seen = null)
    {
        seen ??= new(StringComparer.Ordinal);
        if (!seen.Add(unitName) || ambiguous.Contains(unitName) || !profiles.TryGetValue(unitName, out var definition)) return null;
        if (definition.Unit.Attributes.ContainsKey(attribute))
            return new(Definitions.Array(definition.Unit, attribute), unitName, definition.Root, definition.Sources);
        if (!definition.Unit.Attributes.TryGetValue("inherited", out var inherited)) return null;
        return Attribute(Definitions.Text(inherited), attribute, seen);
    }
    private static object? At(ResolvedAttribute? attribute, uint id) => attribute is { Values.Length: > 0 } ? attribute.Values[Math.Min((long)id, attribute.Values.Length - 1)] : null;
    private static Vector4? IntervalTuple(object? value)
    {
        // TruckLib.Sii parses parenthesized float4 as Quaternion in source
        // W,X,Y,Z order, and all-integer tuples as ValueTuple. These are four time
        // values here, not a rotation: do not normalize or reorder as XYZW.
        if(value is Quaternion q) return new(q.W,q.X,q.Y,q.Z);
        if(value is Vector4 v) return v;
        Vector4? Components(object? a,object? b,object? c,object? d)
        {
            static bool Numeric(object? n)=>n is sbyte or byte or short or ushort or int or uint or long or ulong or float or double or decimal;
            if(!Numeric(a)||!Numeric(b)||!Numeric(c)||!Numeric(d))return null;
            try { return new(Convert.ToSingle(a),Convert.ToSingle(b),Convert.ToSingle(c),Convert.ToSingle(d)); }
            catch(Exception e) when(e is FormatException or InvalidCastException or OverflowException) { return null; }
        }
        if(value is ITuple {Length:4} tuple) return Components(tuple[0],tuple[1],tuple[2],tuple[3]);
        if(value is Array { Length:4 } array) return Components(array.GetValue(0),array.GetValue(1),array.GetValue(2),array.GetValue(3));
        return null;
    }
    private static float? TimingScalar(object? value)
    {
        if(value is not (sbyte or byte or short or ushort or int or uint or long or ulong or float or double or decimal))return null;
        try { return Convert.ToSingle(value); } catch(OverflowException) { return null; }
    }
    private static string? TypeName(string token) => token switch
    {
        "barrier_manual" => "BarrierManualTimed", "barrier_distance" => "BarrierDistanceActivated", "barrier_automatic" => "BarrierAutomatic", "barrier_gas" => "BarrierGas",
        "model_only" => "ModelOnly", "traffic_light" => "TrafficLight", "traffic_light_minor" => "TrafficLightMinor", "traffic_light_major" => "TrafficLightMajor",
        "traffic_light_blockable" => "TrafficLightBlockable", "traffic_light_virtual" => "TrafficLightVirtual", _ => null,
    };
    public SignalRow Resolve(SignalRow raw)
    {
        raw = raw with { type = raw.raw_type ?? raw.type, profile = raw.raw_profile ?? raw.profile };
        if (raw.type != "UseProfile") return raw with { raw_type = raw.type, raw_profile = raw.profile, profile_resolution = "native_ppd", timing_resolved = true };
        if (!overrides.TryGetValue(raw.owner_uid, out var instanceOverride)) return Unknown(raw, "missing_prefab_instance");
        var profile = string.IsNullOrEmpty(instanceOverride) ? raw.profile : instanceOverride;
        if (string.IsNullOrEmpty(profile)) return Unknown(raw, "empty_profile");
        var country = mapCountries[raw.owner_uid]; var baseName = "tr_sem_prof." + profile;
        string[] candidates;
        if (country.Country is { } c)
            candidates = [profiles.ContainsKey(baseName + "." + c) ? baseName + "." + c : baseName];
        else
            candidates = profiles.Keys.Where(n => n.StartsWith(baseName + ".", StringComparison.Ordinal)).Append(baseName).Distinct().Order(StringComparer.Ordinal).ToArray();
        var selected = candidates.Select(name => (Name: name, Type: Attribute(name, "type"), Interval: Attribute(name, "interval"), Cycle: Attribute(name, "cycle"), IdMap: Attribute(name, "id_map"))).ToArray();
        var types = selected.Select(p => At(p.Type, raw.semaphore_id) is { } token ? TypeName(Definitions.Text(token)) : null).ToArray();
        var known = types.Length > 0 && types.All(t => t is not null) && types.Distinct().Count() == 1;
        var intervals = selected.Select(p => IntervalTuple(At(p.Interval, raw.semaphore_id))).ToArray();
        var cycles = selected.Select(p => TimingScalar(At(p.Cycle, raw.semaphore_id))).ToArray();
        var timing = known && intervals.All(v => v.HasValue) && intervals.Distinct().Count() == 1 && cycles.All(v => v.HasValue) && cycles.Distinct().Count() == 1;
        var interval = timing ? intervals[0]!.Value : default; var cycle = timing ? cycles[0]!.Value : 0;
        timing &= float.IsFinite(cycle) && cycle >= 0 && float.IsFinite(interval.X) && float.IsFinite(interval.Y) && float.IsFinite(interval.Z) && float.IsFinite(interval.W)
            && interval.X >= 0 && interval.Y >= 0 && interval.Z >= 0 && interval.W >= 0;
        uint? MapId(object? value) { if (value is null) return null; var number = Convert.ToDouble(value); return double.IsFinite(number) && number >= 0 && number <= uint.MaxValue && number == Math.Truncate(number) ? (uint)number : null; }
        var idMaps = selected.Select(p => MapId(At(p.IdMap, raw.semaphore_id))).ToArray();
        var mapped = idMaps.Distinct().Count() == 1 ? idMaps[0] : null;
        var status = known ? country.Country is not null ? "resolved_profile" : "resolved_type_country_invariant" : "unknown_profile_type_or_country";
        var proof = new { original_profile = raw.profile, effective_profile = profile, instance_override = instanceOverride,
            country, semaphore_id = raw.semaphore_id, resolution = status, resolved_type = known ? types[0] : "Unknown", timing_resolved = timing, resolved_id_map = mapped,
            resolved_interval = timing ? new[] {interval.X,interval.Y,interval.Z,interval.W} : null, resolved_cycle_delay_s = timing ? (float?)cycle : null,
            candidates = selected.Select(p => new { unit = p.Name, type_token = At(p.Type, raw.semaphore_id) is { } token ? Definitions.Text(token) : null,
                type_attribute_unit = p.Type?.Unit, type_attribute_root = p.Type?.Root, source_assets = p.Type?.Sources,
                type_index = p.Type is { Values.Length: > 0 } ? Math.Min((long)raw.semaphore_id, p.Type.Values.Length - 1) : (long?)null,
                type_array_length = p.Type?.Values.Length, interval_unit = p.Interval?.Unit, cycle_unit = p.Cycle?.Unit, id_map_unit = p.IdMap?.Unit }).ToArray() };
        var proofBytes = JsonSerializer.SerializeToUtf8Bytes(proof); var evidenceId = Convert.ToHexStringLower(SHA256.HashData(proofBytes)); evidence[evidenceId] = proof;
        return raw with { type = known ? types[0]! : "Unknown", raw_type = raw.type, raw_profile = raw.profile, effective_profile = profile,
            profile_unit = candidates.Length == 1 ? candidates[0] : null, profile_country = country.Country, profile_resolution = status,
            profile_evidence_id = evidenceId, timing_resolved = timing, resolved_id_map = mapped,
            interval_0 = timing ? interval.X : raw.interval_0, interval_1 = timing ? interval.Y : raw.interval_1,
            interval_2 = timing ? interval.Z : raw.interval_2, interval_3 = timing ? interval.W : raw.interval_3, cycle_delay_s = timing ? cycle : raw.cycle_delay_s };
    }
    private static SignalRow Unknown(SignalRow raw, string reason) => raw with { type = "Unknown", raw_type = raw.type, raw_profile = raw.profile,
        profile_resolution = reason, timing_resolved = false, effective_profile = null, profile_unit = null, profile_country = null, profile_evidence_id = null, resolved_id_map = null };

    internal static void SelfTest()
    {
        static Unit Profile(string name, string[]? types = null, string? parent = null)
        { var u = new Unit("tr_semaphore_profile", name); if(types is not null)u.Attributes["type"]=types.ToList(); if(parent is not null)u.Attributes["inherited"]=parent; return u; }
        static void Check(bool condition, string message) { if(!condition)throw new InvalidOperationException("Signal profile test: "+message); }
        var resolver=new SignalProfileResolver();resolver.Add(Profile("tr_sem_prof.actual",["barrier_manual","barrier_automatic"]),"synthetic",[]);
        resolver.Add(Profile("tr_sem_prof.child",parent:"tr_sem_prof.actual"),"synthetic",[]);
        resolver.Add(Profile("tr_sem_prof.actual.testland",["traffic_light"]),"synthetic",[]);
        resolver.overrides[7]="";resolver.mapCountries[7]=new("otherland","synthetic",0,[]);
        var raw=new SignalRow("7:signal:9",7,9,"UseProfile",0,0,0,0,"child",0,0,0,0,0);
        Check(resolver.Resolve(raw).type=="BarrierAutomatic","inheritance and last array value by native ID");
        resolver.overrides[7]="actual";resolver.mapCountries[7]=new("testland","synthetic",0,[]);
        Check(resolver.Resolve(raw).type=="TrafficLight","actual prefab override and country variant");
        resolver.mapCountries[7]=new(null,"synthetic",0,[]);
        Check(resolver.Resolve(raw).type=="Unknown","ambiguous country cannot be guessed");
        resolver.overrides[7]="ab_toll_name_without_definition";
        Check(resolver.Resolve(raw).type=="Unknown","name does not imply a toll activation rule");
        resolver.Add(Profile("tr_sem_prof.loop",parent:"tr_sem_prof.loop"),"synthetic",[]);resolver.overrides[7]="loop";
        Check(resolver.Resolve(raw).type=="Unknown","inheritance cycle stays unknown");
        Check(resolver.Resolve(raw with {type="TrafficLight"}).type=="TrafficLight","explicit PPD type precedes profile");
        foreach(var tuple in new[]{"(1.0, 3.0, 0.5, 1.5)","(1, 3, 2, 4)"})
        {
            var parsed=SiiFile.Load("SiiNunit\n{\ntr_semaphore_profile : tr_sem_prof.timing\n{\ntype[]: barrier_manual\ninterval[]: "+tuple+"\ncycle[]: 2.5\n}\n}\n");
            resolver.Add(parsed.Units[0],"synthetic",[]);resolver.overrides[7]="timing";resolver.mapCountries[7]=new("otherland","synthetic",0,[]);
            // Each syntax case uses a fresh catalog name to avoid overwriting
            // an intentionally conflict-checked profile declaration.
            var resolved=resolver.Resolve(raw);
            Check(resolved.timing_resolved && resolved.interval_0==1 && resolved.interval_1==3 && resolved.cycle_delay_s==2.5f,"float4 source ordering and typed cycle: "+JsonSerializer.Serialize(new {tuple,parsed_type=At(resolver.Attribute("tr_sem_prof.timing","interval"),9)?.GetType().FullName,parsed=Definitions.Text(At(resolver.Attribute("tr_sem_prof.timing","interval"),9)!),resolved}));
            resolver.profiles.Remove("tr_sem_prof.timing");
        }
        Check(IntervalTuple((1,2,3)) is null && IntervalTuple((1,2,"3",4)) is null,"reject wrong tuple length and nonnumeric components");
        Check(TimingScalar(true) is null && TimingScalar("2.5") is null,"reject nonnumeric cycle types");
        foreach(var bad in new[]{new Vector4(-1,2,3,4),new Vector4(float.NaN,2,3,4),new Vector4(float.PositiveInfinity,2,3,4)})
        {
            var profile=Profile("tr_sem_prof.invalid_time",["barrier_manual"]);profile.Attributes["interval"]=new List<Vector4>{bad};profile.Attributes["cycle"]=new List<float>{0};
            resolver.Add(profile,"synthetic",[]);resolver.overrides[7]="invalid_time";
            Check(!resolver.Resolve(raw).timing_resolved,"reject negative/nonfinite definition time");resolver.profiles.Remove(profile.Name);
        }
        foreach(var bad in new[]{-1f,float.NaN,float.PositiveInfinity})
        {
            var profile=Profile("tr_sem_prof.invalid_cycle",["barrier_manual"]);profile.Attributes["interval"]=new List<Vector4>{new(1,2,3,4)};profile.Attributes["cycle"]=new List<float>{bad};
            resolver.Add(profile,"synthetic",[]);resolver.overrides[7]="invalid_cycle";
            Check(!resolver.Resolve(raw).timing_resolved,"reject negative/nonfinite cycle");resolver.profiles.Remove(profile.Name);
        }
    }
}
