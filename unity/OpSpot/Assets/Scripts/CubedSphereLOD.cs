using System;
using System.Collections.Generic;
using UnityEngine;

namespace OpSpot.Terrain
{
    /// <summary>
    /// Minimal cubed-sphere LOD manager that builds plane-grid tiles projected to a sphere.
    /// Splits/merges tiles using a simple screen-space error estimate.
    /// </summary>
    [DisallowMultipleComponent]
    public class CubedSphereLOD : MonoBehaviour
    {
        [Header("Sphere Shape")]
        [Tooltip("Sphere radius in world units.")]
        public float sphereRadius = 25_000f; // choose a small number to start; adjust to your scale

        [Tooltip("Grid resolution per tile (number of vertices per side). e.g., 33, 65, 129")]
        [Min(2)] public int tileResolution = 65;

        [Header("LOD")]
        [Tooltip("Maximum quadtree depth per face.")]
        [Range(0, 12)] public int maxLevel = 8;

        [Tooltip("Split when screen-space error (in pixels) exceeds this value.")]
        public float splitSsePixels = 50f;

        [Tooltip("Merge when screen-space error (in pixels) falls below this value.")]
        public float mergeSsePixels = 25f;

        [Tooltip("Maximum number of tile splits allowed per frame to avoid spikes.")]
        [Min(1)] public int maxSplitsPerFrame = 16;

        [Tooltip("Hard cap on total active leaf tiles. Excess refinement is suppressed.")]
        [Min(6)] public int maxActiveLeafTiles = 600;

        [Header("Culling")]
        [Tooltip("Enable camera frustum culling for tiles.")]
        public bool enableFrustumCulling = true;

        [Tooltip("Enable simple horizon occlusion culling (planet self-occlusion).")]
        public bool enableHorizonCulling = true;

        [Tooltip("Safety margin (degrees) added to horizon culling to avoid popping at the limb.")]
        [Range(0f, 5f)] public float horizonCullMarginDegrees = 0.5f;

        [Header("Rendering")]
        [Tooltip("Material to use for all tiles.")]
        public Material tileMaterial;

        [Tooltip("Optional explicit camera. If null, uses Camera.main")]
        public Camera targetCamera;

        private readonly List<TileNode> rootTiles = new List<TileNode>(6);
        private readonly Dictionary<string, TileNode> idToNode = new Dictionary<string, TileNode>();

        private void Awake()
        {
            EnsureCamera();
            // Ensure a clean slate (avoids duplicates if something rebuilt us earlier)
            ClearAll();
            DestroyAllChildrenImmediate();
            BuildRoots();
        }

        private void OnValidate()
        {
            if (Application.isPlaying)
            {
                // Rebuild if critical settings changed during play
                ClearAll();
                BuildRoots();
            }
        }

        private void EnsureCamera()
        {
            if (targetCamera == null)
            {
                targetCamera = Camera.main;
            }
        }

        private void BuildRoots()
        {
            rootTiles.Clear();
            idToNode.Clear();
            DestroyAllChildrenImmediate();

            for (int f = 0; f < 6; f++)
            {
                var id = new CubeTileId((CubeFace)f, 0, 0, 0);
                var node = CreateTileNode(id);
                rootTiles.Add(node);
                idToNode[KeyOf(id)] = node;
            }
        }

        private void ClearAll()
        {
            foreach (var kv in idToNode)
            {
                if (kv.Value != null)
                {
                    kv.Value.DestroyImmediate();
                }
            }
            rootTiles.Clear();
            idToNode.Clear();
        }

        private void Update()
        {
            if (targetCamera == null) EnsureCamera();
            if (targetCamera == null) return;

            float focalLengthPixels = ComputeFocalLengthPixels(targetCamera);

            // Frustum planes for visibility culling
            var planes = enableFrustumCulling ? GeometryUtility.CalculateFrustumPlanes(targetCamera) : null;

            // Collect visible leaves that want to split
            List<(TileNode node, float sse)> splitCandidates = new List<(TileNode node, float sse)>(128);
            int activeLeafCount = 0;

            for (int i = 0; i < rootTiles.Count; i++)
            {
                TraverseForLOD(rootTiles[i], planes, focalLengthPixels, splitCandidates, ref activeLeafCount);
            }

            // Prioritize largest SSE first
            splitCandidates.Sort((a, b) => b.sse.CompareTo(a.sse));

            int splits = 0;
            for (int i = 0; i < splitCandidates.Count && splits < maxSplitsPerFrame; i++)
            {
                if (activeLeafCount >= maxActiveLeafTiles) break;
                var candidate = splitCandidates[i];
                if (!candidate.node.HasChildren && candidate.node.Id.Level < maxLevel)
                {
                    SplitNode(candidate.node);
                    splits++;
                    activeLeafCount += 3; // one leaf replaced by four leaves => +3 net
                }
            }
        }

        private void TraverseForLOD(
            TileNode node,
            Plane[] frustumPlanes,
            float focalLengthPixels,
            List<(TileNode node, float sse)> splitCandidates,
            ref int activeLeafCount)
        {
            if (node == null) return;

            // Conservative bounds: sphere approximated by an AABB
            Vector3 center;
            float boundRadius;
            ComputeTileBoundingSphere(node.Id, out center, out boundRadius);

            // Horizon occlusion: cull tiles on the far side of the planet
            if (enableHorizonCulling && IsTileOccludedByPlanet(node.Id))
            {
                node.SetVisible(false);
                if (node.HasChildren) { node.DestroyChildren(); }
                return;
            }

            if (enableFrustumCulling && frustumPlanes != null)
            {
                Bounds bounds = new Bounds(center, Vector3.one * (boundRadius * 2f));
                bool visible = GeometryUtility.TestPlanesAABB(frustumPlanes, bounds);
                if (!visible)
                {
                    node.SetVisible(false);
                    if (node.HasChildren)
                    {
                        node.DestroyChildren();
                    }
                    return;
                }
            }

            if (node.HasChildren)
            {
                node.SetVisible(false);
                foreach (var child in node.Children)
                {
                    TraverseForLOD(child, frustumPlanes, focalLengthPixels, splitCandidates, ref activeLeafCount);
                }

                float parentSse = EstimateTileSsePixels(node.Id, focalLengthPixels);
                if (parentSse < mergeSsePixels)
                {
                    node.DestroyChildren();
                }
            }
            else
            {
                node.SetVisible(true);
                activeLeafCount++;

                if (node.Id.Level < maxLevel)
                {
                    float sse = EstimateTileSsePixels(node.Id, focalLengthPixels);
                    if (sse > splitSsePixels)
                    {
                        splitCandidates.Add((node, sse));
                    }
                }
            }
        }

        private float EstimateTileSsePixels(CubeTileId id, float focalLengthPixels)
        {
            // Geometric proxy: tile edge arc length at this LOD
            float edgeWorld = CubedSphereTile.ApproximateTileEdgeLengthWorld(sphereRadius, id.Level);
            Vector3 center = CubedSphereTile.ComputeTileCenter(id, sphereRadius);
            float distance = Mathf.Max(0.001f, Vector3.Distance(targetCamera.transform.position, center));
            // SSE (pixels) ≈ focal_pixels * (geometric_error_meters / distance)
            float pixels = focalLengthPixels * (edgeWorld / distance);
            return pixels;
        }

        private static float ComputeFocalLengthPixels(Camera cam)
        {
            float fovRad = cam.fieldOfView * Mathf.Deg2Rad;
            float focal = 0.5f * cam.pixelHeight / Mathf.Tan(0.5f * fovRad);
            return focal;
        }

        private void SplitNode(TileNode node)
        {
            if (node.HasChildren) return;

            int childLevel = node.Id.Level + 1;
            int childXBase = node.Id.X << 1;
            int childYBase = node.Id.Y << 1;

            var ids = new CubeTileId[4];
            ids[0] = new CubeTileId(node.Id.Face, childLevel, childXBase, childYBase);
            ids[1] = new CubeTileId(node.Id.Face, childLevel, childXBase + 1, childYBase);
            ids[2] = new CubeTileId(node.Id.Face, childLevel, childXBase, childYBase + 1);
            ids[3] = new CubeTileId(node.Id.Face, childLevel, childXBase + 1, childYBase + 1);

            var children = new TileNode[4];
            for (int i = 0; i < 4; i++)
            {
                children[i] = CreateTileNode(ids[i]);
                idToNode[KeyOf(ids[i])] = children[i];
            }
            node.SetChildren(children);
        }

        private TileNode CreateTileNode(CubeTileId id)
        {
            string name = $"Tile_{id}";
            GameObject go = new GameObject(name);
            go.transform.SetParent(transform, false);

            var mf = go.AddComponent<MeshFilter>();
            var mr = go.AddComponent<MeshRenderer>();
            if (tileMaterial != null)
            {
                mr.sharedMaterial = tileMaterial;
            }
            else
            {
                // Create a simple default material if none provided
                mr.sharedMaterial = new Material(Shader.Find("Universal Render Pipeline/Lit"))
                {
                    color = Color.gray
                };
            }

            Mesh mesh = CubedSphereTile.CreateMesh(id, tileResolution, sphereRadius);
            mf.sharedMesh = mesh;

            var node = new TileNode(id, go, mf, mr);
            return node;
        }

        private static string KeyOf(CubeTileId id)
        {
            return id.ToString();
        }

        private class TileNode
        {
            public readonly CubeTileId Id;
            public readonly GameObject GameObject;
            public readonly MeshFilter MeshFilter;
            public readonly MeshRenderer MeshRenderer;

            public bool HasChildren => _children != null;
            public TileNode[] Children => _children;

            private TileNode[] _children;

            public TileNode(CubeTileId id, GameObject go, MeshFilter mf, MeshRenderer mr)
            {
                Id = id;
                GameObject = go;
                MeshFilter = mf;
                MeshRenderer = mr;
            }

            public void SetChildren(TileNode[] children)
            {
                _children = children;
                // Hide parent geometry when split
                if (GameObject != null) GameObject.SetActive(false);
            }

            public void DestroyChildren()
            {
                if (_children == null) return;
                for (int i = 0; i < _children.Length; i++)
                {
                    _children[i]?.DestroyImmediate();
                }
                _children = null;
                if (GameObject != null) GameObject.SetActive(true);
            }

            public void DestroyImmediate()
            {
                DestroyChildren();
                if (MeshFilter != null && MeshFilter.sharedMesh != null)
                {
                    var m = MeshFilter.sharedMesh;
                    MeshFilter.sharedMesh = null;
#if UNITY_EDITOR
                    UnityEngine.Object.DestroyImmediate(m);
#else
                    UnityEngine.Object.Destroy(m);
#endif
                }
#if UNITY_EDITOR
                if (GameObject != null) UnityEngine.Object.DestroyImmediate(GameObject);
#else
                if (GameObject != null) UnityEngine.Object.Destroy(GameObject);
#endif
            }

            public void SetVisible(bool visible)
            {
                if (GameObject == null) return;
                if (!HasChildren)
                {
                    GameObject.SetActive(visible);
                }
                else
                {
                    GameObject.SetActive(false);
                }
            }
        }

        private void ComputeTileBoundingSphere(CubeTileId id, out Vector3 center, out float radius)
        {
            center = CubedSphereTile.ComputeTileCenter(id, sphereRadius);
            int tilesPerEdge = 1 << id.Level;
            float faceAngle = Mathf.PI * 0.5f; // 90 degrees
            float tileAngle = faceAngle / tilesPerEdge;
            float edgeChord = 2f * sphereRadius * Mathf.Sin(0.5f * tileAngle);
            float halfDiagonal = 0.5f * edgeChord * Mathf.Sqrt(2f);
            radius = halfDiagonal * 1.05f; // margin
        }

        private bool IsTileOccludedByPlanet(CubeTileId id)
        {
            if (targetCamera == null) return false;
            Vector3 cameraPos = targetCamera.transform.position;
            float cameraDistance = cameraPos.magnitude;
            if (cameraDistance <= sphereRadius + 0.001f)
            {
                // At or below the surface: skip to avoid over-culling
                return false;
            }

            Vector3 tileCenter = CubedSphereTile.ComputeTileCenter(id, sphereRadius);
            Vector3 tileDir = tileCenter.normalized; // outward normal at tile center
            Vector3 camDir = cameraPos.normalized;

            // Angle to horizon beta: cos(beta) = R / d
            float cosBeta = Mathf.Clamp(sphereRadius / cameraDistance, 0f, 1f);
            float sinBeta = Mathf.Sqrt(Mathf.Max(0f, 1f - cosBeta * cosBeta));

            // Tile angular radius (half-diagonal arc angle) + small margin
            float tileAngularRadius = ComputeTileAngularRadius(id);
            float r = tileAngularRadius + (horizonCullMarginDegrees * Mathf.Deg2Rad);

            // cos(beta + r) = cosB*cosr - sinB*sinr
            float cosr = Mathf.Cos(r);
            float sinr = Mathf.Sin(r);
            float cosThreshold = cosBeta * cosr - sinBeta * sinr;

            float cosAngle = Vector3.Dot(camDir, tileDir);
            // If angle(camDir, tileDir) > beta + r => occluded (behind horizon)
            return cosAngle < cosThreshold;
        }

        private float ComputeTileAngularRadius(CubeTileId id)
        {
            int tilesPerEdge = 1 << id.Level;
            float faceAngle = Mathf.PI * 0.5f; // 90 degrees per face
            float tileAngle = faceAngle / tilesPerEdge; // radians (edge arc)
            // Edge chord length
            float edgeChord = 2f * sphereRadius * Mathf.Sin(0.5f * tileAngle);
            // Half-diagonal chord length approx
            float halfDiagonalChord = 0.5f * edgeChord * Mathf.Sqrt(2f);
            // Angular radius
            float theta = 2f * Mathf.Asin(Mathf.Clamp(halfDiagonalChord / (2f * sphereRadius), 0f, 1f));
            return theta;
        }

        private void DestroyAllChildrenImmediate()
        {
            // Remove any stray child objects under our transform that are not tracked
            // (e.g., created before a domain reload or inspector change).
            var toDestroy = new List<GameObject>();
            for (int i = 0; i < transform.childCount; i++)
            {
                var child = transform.GetChild(i);
                if (child != null)
                {
                    toDestroy.Add(child.gameObject);
                }
            }
            for (int i = 0; i < toDestroy.Count; i++)
            {
#if UNITY_EDITOR
                UnityEngine.Object.DestroyImmediate(toDestroy[i]);
#else
                UnityEngine.Object.Destroy(toDestroy[i]);
#endif
            }
        }
    }
}


