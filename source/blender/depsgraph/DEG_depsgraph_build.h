/*
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 * The Original Code is Copyright (C) 2013 Blender Foundation.
 * All rights reserved.
 */

/** \file
 * \ingroup depsgraph
 *
 * Public API for Depsgraph
 */

#pragma once

/* ************************************************* */

/* Dependency Graph */
struct Depsgraph;

/* ------------------------------------------------ */

struct CacheFile;
struct Collection;
struct CustomData_MeshMasks;
struct ID;
struct Main;
struct Object;
struct Scene;
struct Simulation;
struct bNodeTree;

#include "BLI_sys_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Graph Building -------------------------------- */

/** Build depsgraph for the given scene layer, and dump results in given graph container. */
void DEG_graph_build_from_view_layer(struct Depsgraph *graph);

/**
 * Build depsgraph for all objects (so also invisible ones) in the given view layer.
 */
void DEG_graph_build_for_all_objects(struct Depsgraph *graph);

/**
 * Special version of builder which produces dependency graph suitable for the render pipeline.
 * It will contain sequencer and compositor (if needed) and all their dependencies.
 */
void DEG_graph_build_for_render_pipeline(struct Depsgraph *graph);

/**
 * Builds minimal dependency graph for compositor preview.
 *
 * Note that compositor editor might have pinned node tree, which is different from scene's node
 * tree.
 */
void DEG_graph_build_for_compositor_preview(struct Depsgraph *graph, struct bNodeTree *nodetree);

void DEG_graph_build_from_ids(struct Depsgraph *graph, struct ID **ids, const int num_ids);

/** Tag relations from the given graph for update. */
void DEG_graph_tag_relations_update(struct Depsgraph *graph);

/** Create or update relations in the specified graph. */
void DEG_graph_relations_update(struct Depsgraph *graph);

/** Tag all relations in the database for update. */
void DEG_relations_tag_update(struct Main *bmain);

/* Add Dependencies  ----------------------------- */

/**
 * Handle for components to define their dependencies from callbacks.
 * This is generated by the depsgraph and passed to dependency callbacks
 * as a symbolic reference to the current DepsNode.
 * All relations will be defined in reference to that node.
 */
struct DepsNodeHandle;

typedef enum eDepsSceneComponentType {
  /* Parameters Component - Default when nothing else fits
   * (i.e. just SDNA property setting). */
  DEG_SCENE_COMP_PARAMETERS,
  /* Animation Component
   * TODO(sergey): merge in with parameters? */
  DEG_SCENE_COMP_ANIMATION,
  /* Sequencer Component (Scene Only). */
  DEG_SCENE_COMP_SEQUENCER,
} eDepsSceneComponentType;

typedef enum eDepsObjectComponentType {
  /* Used in query API, to denote which component caller is interested in. */
  DEG_OB_COMP_ANY,

  /* Parameters Component - Default when nothing else fits
   * (i.e. just SDNA property setting). */
  DEG_OB_COMP_PARAMETERS,
  /* Generic "Proxy-Inherit" Component.
   * TODO(sergey): Also for instancing of subgraphs? */
  DEG_OB_COMP_PROXY,
  /* Animation Component.
   *
   * TODO(sergey): merge in with parameters? */
  DEG_OB_COMP_ANIMATION,
  /* Transform Component (Parenting/Constraints) */
  DEG_OB_COMP_TRANSFORM,
  /* Geometry Component (#Mesh / #DispList). */
  DEG_OB_COMP_GEOMETRY,

  /* Evaluation-Related Outer Types (with Sub-data) */

  /* Pose Component - Owner/Container of Bones Eval */
  DEG_OB_COMP_EVAL_POSE,
  /* Bone Component - Child/Sub-component of Pose */
  DEG_OB_COMP_BONE,

  /* Material Shading Component */
  DEG_OB_COMP_SHADING,
  /* Cache Component */
  DEG_OB_COMP_CACHE,
} eDepsObjectComponentType;

void DEG_add_scene_relation(struct DepsNodeHandle *node_handle,
                            struct Scene *scene,
                            eDepsSceneComponentType component,
                            const char *description);
void DEG_add_object_relation(struct DepsNodeHandle *node_handle,
                             struct Object *object,
                             eDepsObjectComponentType component,
                             const char *description);
void DEG_add_collection_geometry_relation(struct DepsNodeHandle *node_handle,
                                          struct Collection *collection,
                                          const char *description);
void DEG_add_collection_geometry_customdata_mask(struct DepsNodeHandle *node_handle,
                                                 struct Collection *collection,
                                                 const struct CustomData_MeshMasks *masks);
void DEG_add_simulation_relation(struct DepsNodeHandle *node_handle,
                                 struct Simulation *simulation,
                                 const char *description);
void DEG_add_node_tree_output_relation(struct DepsNodeHandle *node_handle,
                                       struct bNodeTree *node_tree,
                                       const char *description);
void DEG_add_bone_relation(struct DepsNodeHandle *handle,
                           struct Object *object,
                           const char *bone_name,
                           eDepsObjectComponentType component,
                           const char *description);
void DEG_add_object_cache_relation(struct DepsNodeHandle *handle,
                                   struct CacheFile *cache_file,
                                   eDepsObjectComponentType component,
                                   const char *description);
/**
 * Adds relation from #DEG_OPCODE_GENERIC_DATABLOCK_UPDATE of a given ID.
 * Is used for such entities as textures and images.
 */
void DEG_add_generic_id_relation(struct DepsNodeHandle *node_handle,
                                 struct ID *id,
                                 const char *description);

/**
 * Special function which is used from modifiers' #updateDepsgraph() callback
 * to indicate that the modifier needs to know transformation of the object
 * which that modifier belongs to.
 * This function will take care of checking which operation is required to
 * have transformation for the modifier, taking into account possible simulation solvers.
 */
void DEG_add_modifier_to_transform_relation(struct DepsNodeHandle *node_handle,
                                            const char *description);

/**
 * Adds relations from the given component of a given object to the given node
 * handle AND the component to the point cache component of the node's ID.
 */
void DEG_add_object_pointcache_relation(struct DepsNodeHandle *node_handle,
                                        struct Object *object,
                                        eDepsObjectComponentType component,
                                        const char *description);

void DEG_add_special_eval_flag(struct DepsNodeHandle *handle, struct ID *id, uint32_t flag);
void DEG_add_customdata_mask(struct DepsNodeHandle *handle,
                             struct Object *object,
                             const struct CustomData_MeshMasks *masks);

struct ID *DEG_get_id_from_handle(struct DepsNodeHandle *node_handle);
struct Depsgraph *DEG_get_graph_from_handle(struct DepsNodeHandle *node_handle);

bool DEG_object_has_geometry_component(struct Object *object);

/* ************************************************ */

#ifdef __cplusplus
} /* extern "C" */
#endif
