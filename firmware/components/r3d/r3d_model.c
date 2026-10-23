/*
 * r3d_model.c - R3DM blob parser / loader
 */
#include <string.h>
#include <stdlib.h>
#include "r3d_model.h"

static uint32_t crc32_update(uint32_t crc, const uint8_t *p, size_t n)
{
    static uint32_t table[256];
    static int ready = 0;
    if (!ready) {
        for (uint32_t i = 0; i < 256; i++) {
            uint32_t c = i;
            for (int k = 0; k < 8; k++) c = (c & 1) ? 0xEDB88320u ^ (c >> 1) : c >> 1;
            table[i] = c;
        }
        ready = 1;
    }
    crc = ~crc;
    while (n--) crc = table[(crc ^ *p++) & 0xFF] ^ (crc >> 8);
    return ~crc;
}

const char *r3d_err_str(r3d_err_t e)
{
    switch (e) {
    case R3D_OK: return "ok";
    case R3D_ERR_MAGIC: return "no model (bad magic) - flash model.bin";
    case R3D_ERR_VERSION: return "model version mismatch - rerun converter";
    case R3D_ERR_SIZE: return "model truncated / partition too small";
    case R3D_ERR_CRC: return "model CRC error";
    case R3D_ERR_NOMEM: return "out of memory";
    case R3D_ERR_FORMAT: return "model format error";
    }
    return "?";
}

static void *default_alloc(size_t n, r3d_mem_t kind) { (void)kind; return malloc(n); }

static const void *copy_section(const void *src, size_t n, r3d_alloc_fn alloc, r3d_mem_t kind)
{
    void *dst = alloc(n, kind);
    if (!dst && kind == R3D_MEM_FAST) dst = alloc(n, R3D_MEM_BIG);
    if (!dst) return NULL;
    memcpy(dst, src, n);
    return dst;
}

r3d_err_t r3d_model_load(r3d_model_t *m, const void *blob, size_t blob_size,
                         r3d_alloc_fn alloc, size_t fast_budget, bool check_crc)
{
    memset(m, 0, sizeof(*m));
    if (!alloc) alloc = default_alloc;
    if (blob_size < sizeof(r3d_blob_header_t)) return R3D_ERR_SIZE;
    const uint8_t *base = (const uint8_t *)blob;
    r3d_blob_header_t h;                       /* copy: blob may be unaligned flash */
    memcpy(&h, blob, sizeof(h));
    if (memcmp(h.magic, "R3DM", 4) != 0) return R3D_ERR_MAGIC;
    if (h.version != R3D_MODEL_VERSION || h.header_size != sizeof(r3d_blob_header_t))
        return R3D_ERR_VERSION;
    if (h.total_size > blob_size) return R3D_ERR_SIZE;
    if (check_crc) {
        uint32_t crc = crc32_update(0, base + h.header_size, h.total_size - h.header_size);
        if (crc != h.crc32) return R3D_ERR_CRC;
    }
    if (h.num_materials == 0 || h.num_materials > R3D_MAX_MATERIALS) return R3D_ERR_FORMAT;
    if (h.num_lods < 1 || h.num_lods > 2) return R3D_ERR_FORMAT;

    m->num_materials = h.num_materials;
    memcpy(m->pos_scale, h.pos_scale, sizeof(m->pos_scale));
    m->radius = h.radius;
    m->ymax = h.ymax;
    memcpy(m->name, h.name, 32);
    m->name[32] = 0;

    for (int l = 0; l < h.num_lods; l++) {
        const r3d_blob_lod_t *bl = &h.lod[l];
        size_t vb = (size_t)bl->num_vertices * sizeof(r3d_blob_vertex_t);
        size_t ib = (size_t)bl->num_triangles * 6;
        size_t tb = (size_t)bl->num_triangles;
        if (bl->off_vertices + vb > h.total_size || bl->off_indices + ib > h.total_size ||
            bl->off_trimat + tb > h.total_size || bl->num_triangles == 0)
            return R3D_ERR_FORMAT;
        r3d_lod_t *L = &m->lod[l];
        L->num_vertices = bl->num_vertices;
        L->num_triangles = (int)bl->num_triangles;
        L->vertices = (const r3d_vertex_t *)copy_section(base + bl->off_vertices, vb, alloc, R3D_MEM_BIG);
        L->indices = copy_section(base + bl->off_indices, ib, alloc, R3D_MEM_BIG);
        L->tri_material = copy_section(base + bl->off_trimat, tb, alloc, R3D_MEM_BIG);
        if (!L->vertices || !L->indices || !L->tri_material) return R3D_ERR_NOMEM;
        /* validate indices / material ids once, so the renderer never reads out of bounds */
        for (int t = 0; t < L->num_triangles * 3; t++)
            if (L->indices[t] >= L->num_vertices) return R3D_ERR_FORMAT;
        for (int t = 0; t < L->num_triangles; t++)
            if (L->tri_material[t] >= m->num_materials) return R3D_ERR_FORMAT;
    }
    if (h.num_lods == 1) { m->lod[1] = m->lod[0]; m->lod_shared = true; }
    for (int k = 0; k < 3; k++) { m->bmin[k] = 1e9f; m->bmax[k] = -1e9f; }
    for (int i = 0; i < m->lod[0].num_vertices; i++) {
        for (int k = 0; k < 3; k++) {
            float v = m->lod[0].vertices[i].p[k] * m->pos_scale[k];
            if (v < m->bmin[k]) m->bmin[k] = v;
            if (v > m->bmax[k]) m->bmax[k] = v;
        }
    }

    size_t fast_used = 0;
    /* largest matcaps (paint, glass, chrome) get the fast RAM first */
    for (int pass = 0; pass < 2; pass++) {
        for (int i = 0; i < m->num_materials; i++) {
            r3d_blob_material_t b;
            memcpy(&b, base + h.off_materials + i * sizeof(b), sizeof(b));
            int big = b.matcap_size >= 128;
            if ((pass == 0) != big) continue;
            if (b.matcap_log2 < 4 || b.matcap_log2 > 8 || b.matcap_size != (1u << b.matcap_log2))
                return R3D_ERR_FORMAT;
            size_t bytes = (size_t)b.matcap_size * b.matcap_size * 2;
            if (b.matcap_offset + bytes > h.total_size) return R3D_ERR_FORMAT;
            r3d_mem_t kind = (fast_used + bytes <= fast_budget) ? R3D_MEM_FAST : R3D_MEM_BIG;
            const void *mc = copy_section(base + b.matcap_offset, bytes, alloc, kind);
            if (!mc) return R3D_ERR_NOMEM;
            if (kind == R3D_MEM_FAST) fast_used += bytes;
            m->mat[i].cls = b.cls;
            m->mat[i].flags = b.flags;
            m->mat[i].log2 = b.matcap_log2;
            m->mat[i].size = b.matcap_size;
            m->mat[i].matcap = (const uint16_t *)mc;
        }
    }
    return R3D_OK;
}

void r3d_model_free(r3d_model_t *m, void (*free_fn)(void *))
{
    if (!free_fn) free_fn = free;
    for (int l = 0; l < 2; l++) {
        if (l == 1 && m->lod_shared) break;
        if (m->lod[l].vertices) free_fn((void *)m->lod[l].vertices);
        if (m->lod[l].indices) free_fn((void *)m->lod[l].indices);
        if (m->lod[l].tri_material) free_fn((void *)m->lod[l].tri_material);
    }
    for (int i = 0; i < R3D_MAX_MATERIALS; i++)
        if (m->mat[i].matcap) free_fn((void *)m->mat[i].matcap);
    memset(m, 0, sizeof(*m));
}

r3d_err_t r3d_model_peek(const void *hdr128, int *max_vertices, int *max_triangles, char name[33])
{
    r3d_blob_header_t h;
    memcpy(&h, hdr128, sizeof(h));
    if (memcmp(h.magic, "R3DM", 4) != 0) return R3D_ERR_MAGIC;
    if (h.version != R3D_MODEL_VERSION || h.header_size != sizeof(r3d_blob_header_t)) return R3D_ERR_VERSION;
    if (h.num_lods < 1 || h.num_lods > 2) return R3D_ERR_FORMAT;
    int mv = 0, mt = 0;
    for (int l = 0; l < h.num_lods; l++) {
        if (h.lod[l].num_vertices > mv) mv = h.lod[l].num_vertices;
        if ((int)h.lod[l].num_triangles > mt) mt = (int)h.lod[l].num_triangles;
    }
    *max_vertices = mv;
    *max_triangles = mt;
    if (name) { memcpy(name, h.name, 32); name[32] = 0; }
    return R3D_OK;
}
