/*
 * r3d_model.h - R3DM binary model format (produced by tools/model_convert.py)
 *
 * Layout (little endian, all sections 16-byte aligned):
 *   header (128 B) | materials (20 B each) | vertices (10 B each) | indices (u16 x3) | matcaps (RGB565)
 */
#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#define R3D_MODEL_VERSION 2

enum {
    R3D_CLS_PAINT = 0, R3D_CLS_GLASS, R3D_CLS_CHROME, R3D_CLS_METAL, R3D_CLS_TIRE,
    R3D_CLS_PLASTIC, R3D_CLS_LIGHT_RED, R3D_CLS_LIGHT_CLEAR, R3D_CLS_INTERIOR,
    R3D_CLS_CARBON, R3D_CLS_GENERIC
};

typedef struct __attribute__((packed)) {
    uint32_t off_vertices;      /* r3d_blob_vertex_t[num_vertices] */
    uint32_t off_indices;       /* uint16_t[3 * num_triangles]     */
    uint32_t off_trimat;        /* uint8_t[num_triangles] material */
    uint16_t num_vertices;
    uint16_t pad;
    uint32_t num_triangles;
} r3d_blob_lod_t;               /* 20 bytes */

typedef struct __attribute__((packed)) {
    char           magic[4];    /* "R3DM" */
    uint16_t       version;
    uint16_t       header_size; /* 128 */
    uint32_t       total_size;
    uint32_t       crc32;       /* zlib crc32 of bytes [header_size, total_size) */
    uint16_t       num_materials;
    uint16_t       num_lods;    /* 1 or 2 (LOD1 = light mesh for the reflection) */
    float          pos_scale[3];/* position = int16 * scale (model units, wheels on y = 0) */
    float          radius;      /* bounding sphere radius around the origin */
    float          ymax;        /* roof height */
    uint32_t       off_materials;
    r3d_blob_lod_t lod[2];
    char           name[32];
    uint8_t        pad[12];
} r3d_blob_header_t;            /* 128 bytes */

typedef struct __attribute__((packed)) {
    uint8_t  cls;
    uint8_t  flags;             /* bit0: double sided */
    uint8_t  matcap_log2;       /* 6 -> 64x64, 7 -> 128x128 */
    uint8_t  pad;
    uint32_t matcap_offset;     /* absolute offset in blob */
    uint16_t matcap_size;
    uint16_t pad2;
} r3d_blob_material_t;          /* 12 bytes */

typedef struct __attribute__((packed)) {
    int16_t p[3];
    int8_t  n[3];
    uint8_t ao;
} r3d_blob_vertex_t;

#define R3D_MAX_MATERIALS 16

#define R3D_MAT_DOUBLE_SIDED 0x01

typedef struct {
    uint8_t         cls;
    uint8_t         log2;
    uint8_t         flags;
    uint16_t        size;
    const uint16_t *matcap;     /* size*size RGB565 (native endian) */
} r3d_material_t;

/* same 10-byte layout as r3d_blob_vertex_t, but 2-byte aligned -> single 16-bit loads */
typedef struct {
    int16_t p[3];
    int8_t  n[3];
    uint8_t ao;
} r3d_vertex_t;
_Static_assert(sizeof(r3d_vertex_t) == sizeof(r3d_blob_vertex_t), "vertex layout");

typedef struct {
    const r3d_vertex_t      *vertices;
    const uint16_t          *indices;      /* 3 per triangle */
    const uint8_t           *tri_material; /* material id per triangle */
    int                      num_vertices;
    int                      num_triangles;
} r3d_lod_t;

typedef struct {
    r3d_lod_t       lod[2];     /* lod[1] == lod[0] when the blob has a single LOD */
    r3d_material_t  mat[R3D_MAX_MATERIALS];
    int             num_materials;
    float           pos_scale[3];
    float           radius, ymax;
    float           bmin[3], bmax[3];   /* axis aligned bounds (model units) */
    char            name[33];
    bool            lod_shared;         /* lod[1] aliases lod[0] (do not free twice) */
} r3d_model_t;

typedef enum {
    R3D_OK = 0,
    R3D_ERR_MAGIC,
    R3D_ERR_VERSION,
    R3D_ERR_SIZE,
    R3D_ERR_CRC,
    R3D_ERR_NOMEM,
    R3D_ERR_FORMAT,
} r3d_err_t;

/* Memory classes the platform can provide */
typedef enum {
    R3D_MEM_FAST = 0,   /* internal SRAM (small, fast, DMA capable) */
    R3D_MEM_BIG  = 1,   /* PSRAM (large, slower)                    */
} r3d_mem_t;

typedef void *(*r3d_alloc_fn)(size_t size, r3d_mem_t kind);

/*
 * Parse a blob (e.g. memory-mapped flash partition).
 * Vertices/indices are copied to R3D_MEM_BIG and matcaps to R3D_MEM_FAST (falling back
 * to BIG) for speed. `alloc` may be NULL (plain malloc, host builds).
 * `fast_budget` = max bytes of fast RAM the matcaps may use.
 */
r3d_err_t r3d_model_load(r3d_model_t *m, const void *blob, size_t blob_size,
                         r3d_alloc_fn alloc, size_t fast_budget, bool check_crc);

/* release everything r3d_model_load() allocated (free_fn matches the alloc used, NULL = free) */
void r3d_model_free(r3d_model_t *m, void (*free_fn)(void *));

/* read only the header: triangle / vertex counts (for sizing the renderer) */
r3d_err_t r3d_model_peek(const void *hdr128, int *max_vertices, int *max_triangles, char name[33]);

const char *r3d_err_str(r3d_err_t e);

#ifdef __cplusplus
}
#endif
