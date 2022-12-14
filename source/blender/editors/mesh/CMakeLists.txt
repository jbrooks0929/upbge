# SPDX-License-Identifier: GPL-2.0-or-later

set(INC
  ../include
  ../uvedit
  ../../blenfont
  ../../blenkernel
  ../../blenlib
  ../../blentranslation
  ../../bmesh
  ../../depsgraph
  ../../draw
  ../../gpu
  ../../imbuf
  ../../makesdna
  ../../makesrna
  ../../render
  ../../windowmanager
  ../../../../intern/clog
  ../../../../intern/glew-mx
  ../../../../intern/guardedalloc
  # RNA_prototypes.h
  ${CMAKE_BINARY_DIR}/source/blender/makesrna
)

set(SRC
  editface.cc
  editmesh_add.c
  editmesh_add_gizmo.c
  editmesh_automerge.c
  editmesh_bevel.c
  editmesh_bisect.c
  editmesh_extrude.c
  editmesh_extrude_screw.c
  editmesh_extrude_spin.c
  editmesh_extrude_spin_gizmo.c
  editmesh_inset.c
  editmesh_intersect.c
  editmesh_knife.c
  editmesh_knife_project.c
  editmesh_loopcut.c
  editmesh_mask_extract.c
  editmesh_path.c
  editmesh_polybuild.c
  editmesh_preselect_edgering.c
  editmesh_preselect_elem.c
  editmesh_rip.c
  editmesh_rip_edge.c
  editmesh_select.c
  editmesh_select_similar.c
  editmesh_tools.c
  editmesh_undo.c
  editmesh_utils.c
  mesh_data.cc
  mesh_mirror.c
  mesh_ops.c
  meshtools.cc

  mesh_intern.h
)

set(LIB
  bf_blenkernel
  bf_blenlib
  bf_windowmanager
)

if(WITH_GAMEENGINE)
  add_definitions(-DWITH_GAMEENGINE)
  list(APPEND INC
    ../../../../extern/recastnavigation
  )

  list(APPEND SRC
    mesh_navmesh.c
  )

  list(APPEND LIB
    extern_recastnavigation
  )
endif()

if(WITH_FREESTYLE)
  add_definitions(-DWITH_FREESTYLE)
endif()

if(WITH_BULLET)
  add_definitions(-DWITH_BULLET)
endif()

if(WITH_GMP)
  add_definitions(-DWITH_GMP)
endif()


blender_add_lib(bf_editor_mesh "${SRC}" "${INC}" "${INC_SYS}" "${LIB}")

# RNA_prototypes.h
add_dependencies(bf_editor_mesh bf_rna)
