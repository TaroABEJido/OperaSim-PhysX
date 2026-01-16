using System.Collections.Generic;
using System.IO;
using System.Text;
using Unity.Collections;
using Unity.Jobs;
using UnityEngine;

public class ScanTerrainAround : MonoBehaviour
{
    public Terrain terrain;

    [Header("Sampling (meters)")]
    public float dx = 0.2f;
    public float dz = 0.2f;

    [Header("Ray settings")]
    public float rayStartHeight = 200f;
    public LayerMask hitMask;
    public int minCommandsPerJob = 64;

    [Header("Output")]
    public string outputPath = "C:/temp/terrain_physx.ply";

    public enum RangeMode
    {
        FullTerrain,
        CenterSizeXZ,
        BoxColliderBounds
    }

    [Header("Range")]
    public RangeMode rangeMode = RangeMode.FullTerrain;

    [Tooltip("RangeMode=CenterSizeXZ のとき有効（ワールド座標）")]
    public Vector3 rangeCenterWorld = Vector3.zero;

    [Tooltip("RangeMode=CenterSizeXZ のとき有効（XZのサイズ[m]）")]
    public Vector2 rangeSizeXZ = new Vector2(10f, 10f);

    [Tooltip("RangeMode=BoxColliderBounds のとき有効（isTriggerでもOK）")]
    public BoxCollider rangeBox;

    public bool clampXZToTerrain = false;

    [ContextMenu("Generate Point Cloud (RaycastCommand)")]
    public void Generate()
    {
        if (terrain == null) terrain = Terrain.activeTerrain;
        if (terrain == null) { Debug.LogError("Terrain not found."); return; }

        Bounds terrainBounds = GetTerrainWorldBounds(terrain);

        // ここが追加：サンプリング範囲を Terrain と交差させて決定
        if (!TryGetSampleBounds(terrainBounds, out Bounds sampleBounds))
        {
            Debug.LogError("Sample bounds does not overlap Terrain bounds.");
            return;
        }

        int nx = Mathf.CeilToInt(sampleBounds.size.x / dx) + 1;
        int nz = Mathf.CeilToInt(sampleBounds.size.z / dz) + 1;
        int n = nx * nz;

        float maxDistance = rayStartHeight + terrainBounds.size.y + 50f;

        var commands = new NativeArray<RaycastCommand>(n, Allocator.TempJob);
        var results  = new NativeArray<RaycastHit>(n, Allocator.TempJob);

        // QueryParameters（Unity 2022.3系で動作する形を維持）
        var qp = QueryParameters.Default;
        qp.layerMask = hitMask;
        qp.hitBackfaces = false;
        qp.hitMultipleFaces = false;
        // 環境によって bool/enum の違いがあるので、あなたの現状に合わせて通る方を使う
        qp.hitTriggers = QueryTriggerInteraction.Ignore;
        // qp.hitTriggers = false;

        float x0 = sampleBounds.min.x;
        float z0 = sampleBounds.min.z;

        int k = 0;
        for (int iz = 0; iz < nz; iz++)
        {
            float z = z0 + iz * dz;
            for (int ix = 0; ix < nx; ix++)
            {
                float x = x0 + ix * dx;

                // 上空から下向き（Terrain上方から開始）
                Vector3 origin = new Vector3(x, terrainBounds.max.y + rayStartHeight, z);
                commands[k] = new RaycastCommand(origin, Vector3.down, qp, maxDistance);
                k++;
            }
        }

        JobHandle handle = RaycastCommand.ScheduleBatch(commands, results, minCommandsPerJob, default);
        handle.Complete();

        var pts = new List<Vector3>(n);
        for (int i = 0; i < n; i++)
        {
            if (results[i].collider != null)
                pts.Add(results[i].point);
        }

        results.Dispose();
        commands.Dispose();



        SaveAsPly(outputPath, pts);
        Debug.Log($"Saved: {outputPath}  points={pts.Count}  bounds={sampleBounds}");
    }

    static Bounds GetTerrainWorldBounds(Terrain t)
    {
        var td = t.terrainData;
        Vector3 tPos = t.transform.position;
        Vector3 size = td.size;
        return new Bounds(tPos + size * 0.5f, size);
    }

    bool TryGetSampleBounds(Bounds terrainBounds, out Bounds sampleBounds)
    {
        Bounds desired;

        switch (rangeMode)
        {
            case RangeMode.FullTerrain:
                sampleBounds = terrainBounds;
                return true;

            case RangeMode.CenterSizeXZ:
                desired = new Bounds(
                    rangeCenterWorld,
                    new Vector3(rangeSizeXZ.x, terrainBounds.size.y, rangeSizeXZ.y)
                );
                break;

            case RangeMode.BoxColliderBounds:
                if (rangeBox == null)
                {
                    sampleBounds = default;
                    return false;
                }
                desired = rangeBox.bounds;
                // Y は不要なので Terrain の高さレンジに合わせる（Ray開始高さ計算を安定させる）
                desired.center = new Vector3(desired.center.x, terrainBounds.center.y, desired.center.z);
                desired.size   = new Vector3(desired.size.x, terrainBounds.size.y, desired.size.z);
                break;

            default:
                sampleBounds = default;
                return false;
        }

        if (!clampXZToTerrain)
        {
            sampleBounds = desired;
            return true;
        }

        // XZ の交差（Terrain外は切り落とす）
        float minX = Mathf.Max(desired.min.x, terrainBounds.min.x);
        float maxX = Mathf.Min(desired.max.x, terrainBounds.max.x);
        float minZ = Mathf.Max(desired.min.z, terrainBounds.min.z);
        float maxZ = Mathf.Min(desired.max.z, terrainBounds.max.z);

        if (minX >= maxX || minZ >= maxZ)
        {
            sampleBounds = default;
            return false;
        }

        Vector3 center = new Vector3((minX + maxX) * 0.5f, terrainBounds.center.y, (minZ + maxZ) * 0.5f);
        Vector3 size   = new Vector3(maxX - minX, terrainBounds.size.y, maxZ - minZ);
        sampleBounds = new Bounds(center, size);
        return true;
    }

    static void SaveAsPly(string path, IReadOnlyList<Vector3> pts)
    {
        var sb = new StringBuilder(pts.Count * 32);
        sb.AppendLine("ply");
        sb.AppendLine("format ascii 1.0");
        sb.AppendLine($"element vertex {pts.Count}");
        sb.AppendLine("property float x");
        sb.AppendLine("property float y");
        sb.AppendLine("property float z");
        sb.AppendLine("end_header");
        foreach (var p0 in pts)
        {
            var p = new Vector3(p0.x, p0.y, -p0.z);   // ← ここで Z 反転（XY 面ミラー補正）
            sb.AppendLine($"{p.x} {p.y} {p.z}");
        }
        File.WriteAllText(path, sb.ToString());
    }
}
