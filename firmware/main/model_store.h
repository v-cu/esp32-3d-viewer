/*
 * model_store.h - one model or a pack of models in the "model" flash partition
 */
#pragma once
#include "r3d_model.h"

int  model_store_init(void);                       /* returns number of valid models */
int  model_store_count(void);
const char *model_store_name(int idx);
void model_store_caps(int *max_vertices, int *max_triangles);
r3d_err_t model_store_load(int idx, r3d_model_t *m);
void model_store_unload(r3d_model_t *m);
