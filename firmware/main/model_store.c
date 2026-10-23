/*
 * model_store.c - models in the "model" flash partition
 *
 * The partition holds either
 *   - one R3DM model (output of tools/model_convert.py), or
 *   - an R3DP pack (tools/pack_models.py) with several models:
 *       "R3DP" u16 version u16 count, then count x { u32 offset, u32 size, char name[24] }
 *       offsets are relative to the start of the partition.
 * Only the active model lives in RAM; switching reads the next one from flash.
 */
#include <string.h>
#include <stdlib.h>
#include "model_store.h"
#include "render.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "esp_partition.h"

static const char *TAG = "models";

#define MAX_MODELS 32

typedef struct __attribute__((packed)) {
    uint32_t offset;
    uint32_t size;
    char     name[24];
} pack_entry_t;

typedef struct {
    uint32_t offset, size;
    char     name[33];
} entry_t;

static const esp_partition_t *s_part;
static entry_t s_ent[MAX_MODELS];
static int s_count;
static int s_max_v, s_max_t;

int model_store_init(void)
{
    s_count = 0;
    s_part = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_ANY, "model");
    if (!s_part) {
        ESP_LOGE(TAG, "no 'model' partition");
        return 0;
    }
    uint8_t hdr[128];
    if (esp_partition_read(s_part, 0, hdr, sizeof(hdr)) != ESP_OK) return 0;

    if (memcmp(hdr, "R3DP", 4) == 0) {
        uint16_t count;
        memcpy(&count, hdr + 6, 2);
        if (count > MAX_MODELS) count = MAX_MODELS;
        for (int i = 0; i < count; i++) {
            pack_entry_t pe;
            if (esp_partition_read(s_part, 8 + i * sizeof(pe), &pe, sizeof(pe)) != ESP_OK) break;
            if (pe.offset + pe.size > s_part->size || pe.size < 128) continue;
            s_ent[s_count].offset = pe.offset;
            s_ent[s_count].size = pe.size;
            memcpy(s_ent[s_count].name, pe.name, 24);
            s_ent[s_count].name[24] = 0;
            s_count++;
        }
    } else if (memcmp(hdr, "R3DM", 4) == 0) {
        uint32_t total;
        memcpy(&total, hdr + 8, 4);
        s_ent[0].offset = 0;
        s_ent[0].size = total;
        s_ent[0].name[0] = 0;
        s_count = 1;
    }

    /* renderer is sized for the biggest model so switching never reallocates it */
    s_max_v = s_max_t = 0;
    int valid = 0;
    for (int i = 0; i < s_count; i++) {
        int v, t;
        char name[33];
        if (esp_partition_read(s_part, s_ent[i].offset, hdr, sizeof(hdr)) != ESP_OK) continue;
        if (r3d_model_peek(hdr, &v, &t, name) != R3D_OK) {
            ESP_LOGW(TAG, "model %d: bad header", i);
            continue;
        }
        if (!s_ent[i].name[0]) strcpy(s_ent[i].name, name);
        if (v > s_max_v) s_max_v = v;
        if (t > s_max_t) s_max_t = t;
        s_ent[valid++] = s_ent[i];
        ESP_LOGI(TAG, "#%d '%s': %d tris, %u KiB", valid, s_ent[valid - 1].name, t,
                 (unsigned)(s_ent[valid - 1].size / 1024));
    }
    s_count = valid;
    return s_count;
}

int model_store_count(void) { return s_count; }
const char *model_store_name(int idx) { return (idx >= 0 && idx < s_count) ? s_ent[idx].name : ""; }
void model_store_caps(int *max_vertices, int *max_triangles) { *max_vertices = s_max_v; *max_triangles = s_max_t; }

static void render_free(void *p) { heap_caps_free(p); }

r3d_err_t model_store_load(int idx, r3d_model_t *m)
{
    if (!s_part || idx < 0 || idx >= s_count) return R3D_ERR_MAGIC;
    const entry_t *e = &s_ent[idx];
    int64_t t0 = esp_timer_get_time();
    /* matcaps of the big glossy materials go to internal RAM (fast random access),
     * as long as enough internal heap stays free for stacks, LVGL and drivers */
    size_t free_int = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    size_t budget = free_int > 72 * 1024 ? free_int - 72 * 1024 : 0;
    if (budget > 96 * 1024) budget = 96 * 1024;

    /* preferred: map the model straight from flash and copy each section once into RAM */
    r3d_err_t err;
    const void *map = NULL;
    esp_partition_mmap_handle_t mh;
    if (esp_partition_mmap(s_part, e->offset, e->size, ESP_PARTITION_MMAP_DATA, &map, &mh) == ESP_OK) {
        err = r3d_model_load(m, map, e->size, render_alloc, budget, true);
        esp_partition_munmap(mh);
    } else {
        /* fallback: read into a staging buffer (PSRAM if possible) */
        uint8_t *blob = heap_caps_malloc(e->size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (!blob) blob = heap_caps_malloc(e->size, MALLOC_CAP_8BIT);
        if (!blob) return R3D_ERR_NOMEM;
        if (esp_partition_read(s_part, e->offset, blob, e->size) != ESP_OK) {
            heap_caps_free(blob);
            return R3D_ERR_SIZE;
        }
        err = r3d_model_load(m, blob, e->size, render_alloc, budget, true);
        heap_caps_free(blob);
    }
    if (err != R3D_OK) {
        r3d_model_free(m, render_free);
        return err;
    }
    ESP_LOGI(TAG, "loaded #%d '%s' (%d tris, %d materials) in %d ms", idx + 1, e->name,
             m->lod[0].num_triangles, m->num_materials, (int)((esp_timer_get_time() - t0) / 1000));
    return R3D_OK;
}

void model_store_unload(r3d_model_t *m) { r3d_model_free(m, render_free); }
