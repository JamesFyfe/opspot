using System;
using UnityEngine;

namespace OpSpot.Terrain
{
    /// <summary>
    /// Identifies a tile on a cubed-sphere quadtree.
    /// </summary>
    [Serializable]
    public struct CubeTileId
    {
        public CubeFace Face; // 0..5
        public int Level;     // 0..N
        public int X;         // 0..(2^L - 1)
        public int Y;         // 0..(2^L - 1)

        public CubeTileId(CubeFace face, int level, int x, int y)
        {
            Face = face;
            Level = level;
            X = x;
            Y = y;
        }

        public override string ToString()
        {
            return $"{Face}/L{Level}/{X},{Y}";
        }
    }

    /// <summary>
    /// One of the six faces of the cubed sphere.
    /// </summary>
    public enum CubeFace
    {
        PositiveX = 0,
        NegativeX = 1,
        PositiveY = 2,
        NegativeY = 3,
        PositiveZ = 4,
        NegativeZ = 5
    }

    /// <summary>
    /// Generates meshes for cubed-sphere tiles (plane grid projected to a sphere).
    /// Keep this stateless and reusable.
    /// </summary>
    public static class CubedSphereTile
    {
        /// <summary>
        /// Create a mesh for the given tile id as a grid of size resolution x resolution,
        /// projected onto a sphere of the provided radius.
        /// </summary>
        public static Mesh CreateMesh(CubeTileId tileId, int resolution, float radius)
        {
            if (resolution < 2)
            {
                resolution = 2;
            }

            int verticesPerSide = resolution;
            int quadCountPerSide = verticesPerSide - 1;

            int vertexCount = verticesPerSide * verticesPerSide;
            int indexCount = quadCountPerSide * quadCountPerSide * 6;

            Vector3[] vertices = new Vector3[vertexCount];
            Vector3[] normals = new Vector3[vertexCount];
            Vector2[] uvs = new Vector2[vertexCount];
            int[] indices = new int[indexCount];

            int idx = 0;
            for (int vy = 0; vy < verticesPerSide; vy++)
            {
                float vLocal = vy / (float)(verticesPerSide - 1); // 0..1 within this tile
                for (int vx = 0; vx < verticesPerSide; vx++)
                {
                    float uLocal = vx / (float)(verticesPerSide - 1); // 0..1 within this tile

                    // Convert tile-local (u,v) to face-global in [0,1]
                    int dim = 1 << tileId.Level; // 2^level
                    float uGlobal = (tileId.X + uLocal) / dim;
                    float vGlobal = (tileId.Y + vLocal) / dim;

                    // Map to [-1, 1] range on cube face
                    float a = uGlobal * 2f - 1f;
                    float b = vGlobal * 2f - 1f;

                    Vector3 cube = FaceUVToCube(tileId.Face, a, b);

                    // Project cube direction to sphere surface
                    Vector3 dir = cube.normalized;
                    Vector3 pos = dir * radius;

                    vertices[idx] = pos;
                    normals[idx] = dir; // good normal for spherical surface
                    // Equirectangular UV from direction for global Earth textures
                    uvs[idx] = DirectionToEquirectangularUV(dir);
                    idx++;
                }
            }

            // Seam fix for equirectangular wrap: make U continuous across the tile
            // Pass 1: rows
            for (int vy = 0; vy < verticesPerSide; vy++)
            {
                int rowStart = vy * verticesPerSide;
                float prevU = uvs[rowStart].x;
                for (int vx = 1; vx < verticesPerSide; vx++)
                {
                    int vi = rowStart + vx;
                    float u = uvs[vi].x;
                    float du = u - prevU;
                    if (du > 0.5f) u -= 1f; else if (du < -0.5f) u += 1f;
                    uvs[vi].x = u;
                    prevU = u;
                }
            }
            // Pass 2: columns
            for (int vx = 0; vx < verticesPerSide; vx++)
            {
                int vi0 = vx;
                float prevU = uvs[vi0].x;
                for (int vy = 1; vy < verticesPerSide; vy++)
                {
                    int vi = vy * verticesPerSide + vx;
                    float u = uvs[vi].x;
                    float du = u - prevU;
                    if (du > 0.5f) u -= 1f; else if (du < -0.5f) u += 1f;
                    uvs[vi].x = u;
                    prevU = u;
                }
            }

            int ii = 0;
            for (int y = 0; y < quadCountPerSide; y++)
            {
                for (int x = 0; x < quadCountPerSide; x++)
                {
                    int i0 = y * verticesPerSide + x;
                    int i1 = i0 + 1;
                    int i2 = i0 + verticesPerSide;
                    int i3 = i2 + 1;

                    // Two triangles per quad with outward-facing CCW winding
                    indices[ii++] = i0; // tri 1: i0, i1, i2
                    indices[ii++] = i1;
                    indices[ii++] = i2;

                    indices[ii++] = i1; // tri 2: i1, i3, i2
                    indices[ii++] = i3;
                    indices[ii++] = i2;
                }
            }

            Mesh mesh = new Mesh();
            // 129x129 -> 16641 vertices < 65535, but allow larger resolutions if user wants.
            if (vertexCount > 65000)
            {
                mesh.indexFormat = UnityEngine.Rendering.IndexFormat.UInt32;
            }
            mesh.name = $"Tile_{tileId}";
            mesh.SetVertices(vertices);
            mesh.SetNormals(normals);
            mesh.SetUVs(0, uvs);
            mesh.SetIndices(indices, MeshTopology.Triangles, 0, true);
            mesh.RecalculateBounds(); // normals already set
            return mesh;
        }

        /// <summary>
        /// Returns the world-space center of the tile on the sphere (by sampling the center UV).
        /// </summary>
        public static Vector3 ComputeTileCenter(CubeTileId tileId, float radius)
        {
            int dim = 1 << tileId.Level; // 2^level
            float uGlobal = (tileId.X + 0.5f) / dim;
            float vGlobal = (tileId.Y + 0.5f) / dim;

            float a = uGlobal * 2f - 1f;
            float b = vGlobal * 2f - 1f;
            Vector3 cube = FaceUVToCube(tileId.Face, a, b);
            Vector3 dir = cube.normalized;
            return dir * radius;
        }

        /// <summary>
        /// Approximate tile edge length in world units along the sphere surface.
        /// Each cube face spans 90 degrees (pi/2 radians). Per level subdivision halves the span.
        /// </summary>
        public static float ApproximateTileEdgeLengthWorld(float radius, int level)
        {
            float faceAngle = Mathf.PI * 0.5f; // 90 degrees per face
            float tilesPerEdge = 1 << level;   // 2^level tiles along an edge
            float tileAngle = faceAngle / tilesPerEdge; // radians
            // Arc length s = r * theta
            return radius * tileAngle;
        }

        /// <summary>
        /// Map face-local coordinates in [-1,1]x[-1,1] to cube space with one axis fixed to +/-1.
        /// </summary>
        private static Vector3 FaceUVToCube(CubeFace face, float a, float b)
        {
            switch (face)
            {
                case CubeFace.PositiveX: return new Vector3(1f, b, -a);
                case CubeFace.NegativeX: return new Vector3(-1f, b, a);
                case CubeFace.PositiveY: return new Vector3(a, 1f, -b);
                case CubeFace.NegativeY: return new Vector3(a, -1f, b);
                case CubeFace.PositiveZ: return new Vector3(a, b, 1f);
                case CubeFace.NegativeZ: return new Vector3(-a, b, -1f);
                default: return Vector3.zero;
            }
        }

        /// <summary>
        /// Convert a unit direction vector to equirectangular UVs (lon/lat mapping).
        /// u in [0,1) wraps at +/-180° longitude; v in [0,1].
        /// </summary>
        private static Vector2 DirectionToEquirectangularUV(in Vector3 dir)
        {
            // Longitude lambda in [-pi, pi]
            float lambda = Mathf.Atan2(dir.x, dir.z);
            // Latitude phi in [-pi/2, pi/2]
            float phi = Mathf.Asin(Mathf.Clamp(dir.y, -1f, 1f));

            float u = 0.5f - (lambda / (2f * Mathf.PI));
            // wrap to [0,1)
            u = u - Mathf.Floor(u);
            float v = 0.5f + (phi / Mathf.PI);
            return new Vector2(u, v);
        }
    }
}


