/*
 * ***** BEGIN GPL LICENSE BLOCK *****
 *
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
 * The Original Code is Copyright (C) 2001-2002 by NaN Holding BV.
 * All rights reserved.
 *
 * The Original Code is: all of this file.
 *
 * Contributor(s): none yet.
 *
 * ***** END GPL LICENSE BLOCK *****
 */

/** \file gameengine/Converter/BL_ArmatureObject.cpp
 *  \ingroup bgeconv
 */

#include "BL_ArmatureObject.h"

#include "BKE_action.h"
#include "BKE_animsys.h"
#include "BKE_armature.h"
#include "BKE_constraint.h"
#include "BKE_context.h"
#include "BKE_DerivedMesh.h"
#include "BKE_layer.h"
#include "BKE_lib_id.h"
#include "BKE_scene.h"
#include "BLI_task.h"
#include "DNA_armature_types.h"
#include "DNA_mesh_types.h"
#include "DNA_modifier_types.h"
#include "MEM_guardedalloc.h"
#include "RNA_access.h"

#include "BL_Action.h"
#include "BL_BlenderSceneConverter.h"
#include "KX_Globals.h"

/**
 * Move here pose function for game engine so that we can mix with GE objects
 * Principle is as follow:
 * Use Blender structures so that BKE_pose_where_is can be used unchanged
 * Copy the constraint so that they can be enabled/disabled/added/removed at runtime
 * Don't copy the constraints for the pose used by the Action actuator, it does not need them.
 * Scan the constraint structures so that the KX equivalent of target objects are identified and
 * stored in separate list.
 * When it is about to evaluate the pose, set the KX object position in the obmat of the
 * corresponding Blender objects and restore after the evaluation.
 */
static void game_copy_pose(bPose **dst, bPose *src, int copy_constraint)
{
  /* The game engine copies the current armature pose and then swaps
   * the object pose pointer. this makes it possible to change poses
   * without affecting the original blender data. */

  if (!src) {
    *dst = nullptr;
    return;
  }
  else if (*dst == src) {
    CM_Warning("game_copy_pose source and target are the same");
    *dst = nullptr;
    return;
  }

  bPose *out = (bPose *)MEM_dupallocN(src);
  out->chanhash = nullptr;
  out->agroups.first = out->agroups.last = nullptr;
  out->ikdata = nullptr;
  out->ikparam = MEM_dupallocN(src->ikparam);
  // out->flag |= POSE_GAME_ENGINE;
  BLI_duplicatelist(&out->chanbase, &src->chanbase);

  /* remap pointers */
  GHash *ghash = BLI_ghash_new(BLI_ghashutil_ptrhash, BLI_ghashutil_ptrcmp, "game_copy_pose gh");

  bPoseChannel *pchan = (bPoseChannel *)src->chanbase.first;
  bPoseChannel *outpchan = (bPoseChannel *)out->chanbase.first;
  for (; pchan; pchan = pchan->next, outpchan = outpchan->next) {
    BLI_ghash_insert(ghash, pchan, outpchan);
  }

  for (pchan = (bPoseChannel *)out->chanbase.first; pchan; pchan = pchan->next) {
    pchan->parent = (bPoseChannel *)BLI_ghash_lookup(ghash, pchan->parent);
    pchan->child = (bPoseChannel *)BLI_ghash_lookup(ghash, pchan->child);

    if (copy_constraint) {
      ListBase listb;
      // copy all constraint for backward compatibility
      // BKE_constraints_copy nullptrs listb, no need to make extern for this operation.
      BKE_constraints_copy(&listb, &pchan->constraints, false);
      pchan->constraints = listb;
    }
    else {
      BLI_listbase_clear(&pchan->constraints);
    }

    if (pchan->custom) {
      id_us_plus(&pchan->custom->id);
    }

    // fails to link, props are not used in the BGE yet.
#if 0
		if (pchan->prop) {
			pchan->prop = IDP_CopyProperty(pchan->prop);
		}
#endif
    pchan->prop = nullptr;
  }

  BLI_ghash_free(ghash, nullptr, nullptr);
  // set acceleration structure for channel lookup
  BKE_pose_channels_hash_ensure(out);
  *dst = out;
}

// Only allowed for Poses with identical channels.
static void game_blend_poses(bPose *dst, bPose *src, float srcweight, short mode)
{
  float dstweight;

  if (mode == BL_Action::ACT_BLEND_BLEND) {
    dstweight = 1.0f - srcweight;
  }
  else if (mode == BL_Action::ACT_BLEND_ADD) {
    dstweight = 1.0f;
  }
  else {
    dstweight = 1.0f;
  }

  bPoseChannel *schan = (bPoseChannel *)src->chanbase.first;
  for (bPoseChannel *dchan = (bPoseChannel *)dst->chanbase.first; dchan;
       dchan = (bPoseChannel *)dchan->next, schan = (bPoseChannel *)schan->next) {
    // always blend on all channels since we don't know which one has been set
    /* quat interpolation done separate */
    if (schan->rotmode == ROT_MODE_QUAT) {
      float dquat[4], squat[4];

      copy_qt_qt(dquat, dchan->quat);
      copy_qt_qt(squat, schan->quat);
      // Normalize quaternions so that interpolation/multiplication result is correct.
      normalize_qt(dquat);
      normalize_qt(squat);

      if (mode == BL_Action::ACT_BLEND_BLEND) {
        interp_qt_qtqt(dchan->quat, dquat, squat, srcweight);
      }
      else {
        pow_qt_fl_normalized(squat, srcweight);
        mul_qt_qtqt(dchan->quat, dquat, squat);
      }

      normalize_qt(dchan->quat);
    }

    for (unsigned short i = 0; i < 3; i++) {
      /* blending for loc and scale are pretty self-explanatory... */
      dchan->loc[i] = (dchan->loc[i] * dstweight) + (schan->loc[i] * srcweight);
      dchan->size[i] = 1.0f + ((dchan->size[i] - 1.0f) * dstweight) +
                       ((schan->size[i] - 1.0f) * srcweight);

      /* euler-rotation interpolation done here instead... */
      // FIXME: are these results decent?
      if (schan->rotmode) {
        dchan->eul[i] = (dchan->eul[i] * dstweight) + (schan->eul[i] * srcweight);
      }
    }
    for (bConstraint *dcon = (bConstraint *)dchan->constraints.first,
                     *scon = (bConstraint *)schan->constraints.first;
         dcon && scon;
         dcon = dcon->next, scon = scon->next) {
      /* no 'add' option for constraint blending */
      dcon->enforce = dcon->enforce * (1.0f - srcweight) + scon->enforce * srcweight;
    }
  }

  /* this pose is now in src time */
  dst->ctime = src->ctime;
}

BL_ArmatureObject::BL_ArmatureObject(void *sgReplicationInfo,
                                     SG_Callbacks callbacks,
                                     Object *armature,
                                     Scene *scene)
    : KX_GameObject(sgReplicationInfo, callbacks),
      m_scene(scene),
      m_lastframe(0.0),
      m_drawDebug(false),
      m_lastapplyframe(0.0)
{
  m_controlledConstraints = new EXP_ListValue<BL_ArmatureConstraint>();
  m_poseChannels = new EXP_ListValue<BL_ArmatureChannel>();

  // Keep a copy of the original armature so we can fix drivers later
  m_origObjArma = armature;
  m_objArma = m_origObjArma;  // BKE_object_copy(bmain, armature);
  // m_objArma->data = BKE_armature_copy(bmain, (bArmature *)armature->data);
  // During object replication ob->data is increase, we decrease it now because we get a copy.
  // id_us_min(&((bArmature *)m_origObjArma->data)->id);
  // need this to get iTaSC working ok in the BGE
  // m_objArma->pose->flag |= POSE_GAME_ENGINE;
  memcpy(m_obmat, m_objArma->obmat, sizeof(m_obmat));
}

BL_ArmatureObject::~BL_ArmatureObject()
{
  m_poseChannels->Release();
  m_controlledConstraints->Release();

  // if (m_objArma) {
  //	BKE_id_free(bmain, m_objArma->data);
  //	/* avoid BKE_libblock_free(bmain, m_objArma)
  //	   try to access m_objArma->data */
  //	m_objArma->data = nullptr;
  //	BKE_id_free(bmain, m_objArma);
  //}
}

typedef struct bPoseChanDeform {
  Mat4 *b_bone_mats;
  DualQuat *dual_quat;
  DualQuat *b_bone_dual_quats;
} bPoseChanDeform;

typedef struct ArmatureBBoneDefmatsData {
  bPoseChanDeform *pdef_info_array;
  DualQuat *dualquats;
  bool use_quaternion;
} ArmatureBBoneDefmatsData;

static void pchan_b_bone_defmats(bPoseChannel *pchan,
                                 bPoseChanDeform *pdef_info,
                                 const bool use_quaternion)
{
  Bone *bone = pchan->bone;
  Mat4 b_bone[MAX_BBONE_SUBDIV], b_bone_rest[MAX_BBONE_SUBDIV];
  Mat4 *b_bone_mats;
  DualQuat *b_bone_dual_quats = NULL;
  int a;

  BKE_pchan_bbone_spline_setup(pchan, 0, 1, b_bone);
  BKE_pchan_bbone_spline_setup(pchan, 1, 0, b_bone_rest);

  /* allocate b_bone matrices and dual quats */
  b_bone_mats = (Mat4 *)MEM_mallocN((1 + bone->segments) * sizeof(Mat4), "BBone defmats");
  pdef_info->b_bone_mats = b_bone_mats;

  if (use_quaternion) {
    b_bone_dual_quats = (DualQuat *)MEM_mallocN((bone->segments) * sizeof(DualQuat), "BBone dqs");
    pdef_info->b_bone_dual_quats = b_bone_dual_quats;
  }

  /* first matrix is the inverse arm_mat, to bring points in local bone space
   * for finding out which segment it belongs to */
  invert_m4_m4(b_bone_mats[0].mat, bone->arm_mat);

  /* then we make the b_bone_mats:
   * - first transform to local bone space
   * - translate over the curve to the bbone mat space
   * - transform with b_bone matrix
   * - transform back into global space */

  for (a = 0; a < bone->segments; a++) {
    float tmat[4][4];

    invert_m4_m4(tmat, b_bone_rest[a].mat);
    mul_m4_series(b_bone_mats[a + 1].mat,
                  pchan->chan_mat,
                  bone->arm_mat,
                  b_bone[a].mat,
                  tmat,
                  b_bone_mats[0].mat);

    if (use_quaternion)
      mat4_to_dquat(&b_bone_dual_quats[a], bone->arm_mat, b_bone_mats[a + 1].mat);
  }
}

static void armature_bbone_defmats_cb(void *userdata, Link *iter, int index)
{
  ArmatureBBoneDefmatsData *data = (ArmatureBBoneDefmatsData *)userdata;
  bPoseChannel *pchan = (bPoseChannel *)iter;

  if (!(pchan->bone->flag & BONE_NO_DEFORM)) {
    bPoseChanDeform *pdef_info = &data->pdef_info_array[index];
    const bool use_quaternion = data->use_quaternion;

    if (pchan->bone->segments > 1) {
      pchan_b_bone_defmats(pchan, pdef_info, use_quaternion);
    }

    if (use_quaternion) {
      pdef_info->dual_quat = &data->dualquats[index];
      mat4_to_dquat(pdef_info->dual_quat, pchan->bone->arm_mat, pchan->chan_mat);
    }
  }
}

int defgroup_name_index(Object *ob, const char *name)
{
  return (name) ? BLI_findstringindex(&ob->defbase, name, offsetof(bDeformGroup, name)) : -1;
}

MDeformWeight *defvert_find_index(const MDeformVert *dvert, const int defgroup)
{
  if (dvert && defgroup >= 0) {
    MDeformWeight *dw = dvert->dw;
    unsigned int i;

    for (i = dvert->totweight; i != 0; i--, dw++) {
      if (dw->def_nr == defgroup) {
        return dw;
      }
    }
  }
  else {
    BLI_assert(0);
  }

  return NULL;
}

static void b_bone_deform(
    bPoseChanDeform *pdef_info, Bone *bone, float co[3], DualQuat *dq, float defmat[3][3])
{
  Mat4 *b_bone = pdef_info->b_bone_mats;
  float(*mat)[4] = b_bone[0].mat;
  float segment, y;
  int a;

  /* need to transform co back to bonespace, only need y */
  y = mat[0][1] * co[0] + mat[1][1] * co[1] + mat[2][1] * co[2] + mat[3][1];

  /* now calculate which of the b_bones are deforming this */
  segment = bone->length / ((float)bone->segments);
  a = (int)(y / segment);

  /* note; by clamping it extends deform at endpoints, goes best with
   * straight joints in restpos. */
  CLAMP(a, 0, bone->segments - 1);

  if (dq) {
    copy_dq_dq(dq, &(pdef_info->b_bone_dual_quats)[a]);
  }
  else {
    mul_m4_v3(b_bone[a + 1].mat, co);

    if (defmat) {
      copy_m3_m4(defmat, b_bone[a + 1].mat);
    }
  }
}

static void pchan_deform_mat_add(bPoseChannel *pchan,
                                 float weight,
                                 float bbonemat[3][3],
                                 float mat[3][3])
{
  float wmat[3][3];

  if (pchan->bone->segments > 1)
    copy_m3_m3(wmat, bbonemat);
  else
    copy_m3_m4(wmat, pchan->chan_mat);

  mul_m3_fl(wmat, weight);
  add_m3_m3m3(mat, mat, wmat);
}

static void pchan_bone_deform(bPoseChannel *pchan,
                              bPoseChanDeform *pdef_info,
                              float weight,
                              float vec[3],
                              DualQuat *dq,
                              float mat[3][3],
                              const float co[3],
                              float *contrib)
{
  float cop[3], bbonemat[3][3];
  DualQuat bbonedq;

  if (!weight)
    return;

  copy_v3_v3(cop, co);

  if (vec) {
    if (pchan->bone->segments > 1)
      /* applies on cop and bbonemat */
      b_bone_deform(pdef_info, pchan->bone, cop, NULL, (mat) ? bbonemat : NULL);
    else
      mul_m4_v3(pchan->chan_mat, cop);

    vec[0] += (cop[0] - co[0]) * weight;
    vec[1] += (cop[1] - co[1]) * weight;
    vec[2] += (cop[2] - co[2]) * weight;

    if (mat)
      pchan_deform_mat_add(pchan, weight, bbonemat, mat);
  }
  else {
    if (pchan->bone->segments > 1) {
      b_bone_deform(pdef_info, pchan->bone, cop, &bbonedq, NULL);
      add_weighted_dq_dq(dq, &bbonedq, weight);
    }
    else
      add_weighted_dq_dq(dq, pdef_info->dual_quat, weight);
  }

  (*contrib) += weight;
}

float defvert_find_weight(const struct MDeformVert *dvert, const int defgroup)
{
  MDeformWeight *dw = defvert_find_index(dvert, defgroup);
  return dw ? dw->weight : 0.0f;
}

static float dist_bone_deform(bPoseChannel *pchan,
                              bPoseChanDeform *pdef_info,
                              float vec[3],
                              DualQuat *dq,
                              float mat[3][3],
                              const float co[3])
{
  Bone *bone = pchan->bone;
  float fac, contrib = 0.0;
  float cop[3], bbonemat[3][3];
  DualQuat bbonedq;

  if (bone == NULL)
    return 0.0f;

  copy_v3_v3(cop, co);

  fac = distfactor_to_bone(
      cop, bone->arm_head, bone->arm_tail, bone->rad_head, bone->rad_tail, bone->dist);

  if (fac > 0.0f) {
    fac *= bone->weight;
    contrib = fac;
    if (contrib > 0.0f) {
      if (vec) {
        if (bone->segments > 1)
          /* applies on cop and bbonemat */
          b_bone_deform(pdef_info, bone, cop, NULL, (mat) ? bbonemat : NULL);
        else
          mul_m4_v3(pchan->chan_mat, cop);

        /* Make this a delta from the base position */
        sub_v3_v3(cop, co);
        madd_v3_v3fl(vec, cop, fac);

        if (mat)
          pchan_deform_mat_add(pchan, fac, bbonemat, mat);
      }
      else {
        if (bone->segments > 1) {
          b_bone_deform(pdef_info, bone, cop, &bbonedq, NULL);
          add_weighted_dq_dq(dq, &bbonedq, fac);
        }
        else
          add_weighted_dq_dq(dq, pdef_info->dual_quat, fac);
      }
    }
  }

  return contrib;
}

static short get_deformflags(Object *bmeshobj)
{
  short flags = ARM_DEF_VGROUP;

  ModifierData *md;
  for (md = (ModifierData *)bmeshobj->modifiers.first; md; md = md->next) {
    if (md->type == eModifierType_Armature) {
      flags |= ((ArmatureModifierData *)md)->deformflag;
      break;
    }
  }

  return flags;
}

void BL_ArmatureObject::armature_deform_verts(Object *armOb,
                           Object *target,
                           DerivedMesh *dm,
                           float (*vertexCos)[3],
                           float (*defMats)[3][3],
                           int numVerts,
                           float (*prevCos)[3],
                           const char *defgrp_name)
{
  float obmat[4][4];  // the original object matrix
    // save matrix first
  copy_m4_m4(obmat, target->obmat);
  // set reference matrix (ref armature obmat)
  copy_m4_m4(target->obmat, m_obmat);

  bPoseChanDeform *pdef_info_array;
  bPoseChanDeform *pdef_info = NULL;
  bArmature *arm = (bArmature *)armOb->data;
  bPoseChannel *pchan, **defnrToPC = NULL;
  int *defnrToPCIndex = NULL;
  MDeformVert *dverts = NULL;
  bDeformGroup *dg;
  DualQuat *dualquats = NULL;

  int deformflag = get_deformflags(target);

  float obinv[4][4], premat[4][4], postmat[4][4];
  const bool use_envelope = (deformflag & ARM_DEF_ENVELOPE) != 0;
  const bool use_quaternion = (deformflag & ARM_DEF_QUATERNION) != 0;
  const bool invert_vgroup = (deformflag & ARM_DEF_INVERT_VGROUP) != 0;
  int defbase_tot = 0;       /* safety for vertexgroup index overflow */
  int i, target_totvert = 0; /* safety for vertexgroup overflow */
  bool use_dverts = false;
  int armature_def_nr;
  int totchan;

  /* in editmode, or not an armature */
  if (arm->edbo || (armOb->pose == NULL)) {
    return;
  }

  if ((armOb->pose->flag & POSE_RECALC) != 0) {
    printf("ERROR! Trying to evaluate influence of armature '%s' which needs Pose recalc!",
           armOb->id.name);
    BLI_assert(0);
  }

  invert_m4_m4(obinv, target->obmat);
  copy_m4_m4(premat, target->obmat);
  mul_m4_m4m4(postmat, obinv, armOb->obmat);
  invert_m4_m4(premat, postmat);

  /* bone defmats are already in the channels, chan_mat */

  /* initialize B_bone matrices and dual quaternions */
  totchan = BLI_listbase_count(&armOb->pose->chanbase);

  if (use_quaternion) {
    dualquats = (DualQuat *)MEM_callocN(sizeof(DualQuat) * totchan, "dualquats");
  }

  pdef_info_array = (bPoseChanDeform *)MEM_callocN(sizeof(bPoseChanDeform) * totchan, "bPoseChanDeform");

  ArmatureBBoneDefmatsData data;
  data.pdef_info_array = pdef_info_array;
  data.dualquats = dualquats;
  data.use_quaternion = use_quaternion;
  TaskParallelSettings settings;
  BLI_parallel_range_settings_defaults(&settings);
  BLI_task_parallel_listbase(
      &armOb->pose->chanbase, &data, (TaskParallelIteratorFunc)armature_bbone_defmats_cb, &settings);

  /* get the def_nr for the overall armature vertex group if present */
  armature_def_nr = defgroup_name_index(target, defgrp_name);

  if (ELEM(target->type, OB_MESH, OB_LATTICE)) {
    defbase_tot = BLI_listbase_count(&target->defbase);

    if (target->type == OB_MESH) {
      Mesh *me = (Mesh *)target->data;
      dverts = me->dvert;
      if (dverts)
        target_totvert = me->totvert;
    }
    else {
      /*Lattice *lt = target->data;
      dverts = lt->dvert;
      if (dverts)
        target_totvert = lt->pntsu * lt->pntsv * lt->pntsw;*/
    }
  }

  /* get a vertex-deform-index to posechannel array */
  if (deformflag & ARM_DEF_VGROUP) {
    if (ELEM(target->type, OB_MESH, OB_LATTICE)) {
      /* if we have a DerivedMesh, only use dverts if it has them */
      if (dm) {
        use_dverts = (dm->getVertDataArray(dm, CD_MDEFORMVERT) != NULL);
      }
      else if (dverts) {
        use_dverts = true;
      }

      if (use_dverts) {
        defnrToPC = (bPoseChannel **)MEM_callocN(sizeof(*defnrToPC) * defbase_tot, "defnrToBone");
        defnrToPCIndex = (int *)MEM_callocN(sizeof(*defnrToPCIndex) * defbase_tot, "defnrToIndex");
        /* TODO(sergey): Some considerations here:
         *
         * - Make it more generic function, maybe even keep together with chanhash.
         * - Check whether keeping this consistent across frames gives speedup.
         * - Don't use hash for small armatures.
         */
        GHash *idx_hash = BLI_ghash_ptr_new("pose channel index by name");
        int pchan_index = 0;
        for (pchan = (bPoseChannel *)armOb->pose->chanbase.first; pchan != NULL;
             pchan = pchan->next, ++pchan_index) {
          BLI_ghash_insert(idx_hash, pchan, POINTER_FROM_INT(pchan_index));
        }
        for (i = 0, dg = (bDeformGroup *)target->defbase.first; dg; i++, dg = dg->next) {
          defnrToPC[i] = BKE_pose_channel_find_name(armOb->pose, dg->name);
          /* exclude non-deforming bones */
          if (defnrToPC[i]) {
            if (defnrToPC[i]->bone->flag & BONE_NO_DEFORM) {
              defnrToPC[i] = NULL;
            }
            else {
              defnrToPCIndex[i] = POINTER_AS_INT(BLI_ghash_lookup(idx_hash, defnrToPC[i]));
            }
          }
        }
        BLI_ghash_free(idx_hash, NULL, NULL);
      }
    }
  }

  for (i = 0; i < numVerts; i++) {
    MDeformVert *dvert;
    DualQuat sumdq, *dq = NULL;
    float *co, dco[3];
    float sumvec[3], summat[3][3];
    float *vec = NULL, (*smat)[3] = NULL;
    float contrib = 0.0f;
    float armature_weight = 1.0f; /* default to 1 if no overall def group */
    float prevco_weight = 1.0f;   /* weight for optional cached vertexcos */

    if (use_quaternion) {
      memset(&sumdq, 0, sizeof(DualQuat));
      dq = &sumdq;
    }
    else {
      sumvec[0] = sumvec[1] = sumvec[2] = 0.0f;
      vec = sumvec;

      if (defMats) {
        zero_m3(summat);
        smat = summat;
      }
    }

    if (use_dverts || armature_def_nr != -1) {
      if (dm)
        dvert = (MDeformVert *)dm->getVertData(dm, i, CD_MDEFORMVERT);
      else if (dverts && i < target_totvert)
        dvert = dverts + i;
      else
        dvert = NULL;
    }
    else
      dvert = NULL;

    if (armature_def_nr != -1 && dvert) {
      armature_weight = defvert_find_weight(dvert, armature_def_nr);

      if (invert_vgroup)
        armature_weight = 1.0f - armature_weight;

      /* hackish: the blending factor can be used for blending with prevCos too */
      if (prevCos) {
        prevco_weight = armature_weight;
        armature_weight = 1.0f;
      }
    }

    /* check if there's any  point in calculating for this vert */
    if (armature_weight == 0.0f)
      continue;

    /* get the coord we work on */
    co = prevCos ? prevCos[i] : vertexCos[i];

    /* Apply the object's matrix */
    mul_m4_v3(premat, co);

    if (use_dverts && dvert && dvert->totweight) { /* use weight groups ? */
      MDeformWeight *dw = dvert->dw;
      int deformed = 0;
      unsigned int j;

      for (j = dvert->totweight; j != 0; j--, dw++) {
        const int index = dw->def_nr;
        if (index >= 0 && index < defbase_tot && (pchan = defnrToPC[index])) {
          float weight = dw->weight;
          Bone *bone = pchan->bone;
          pdef_info = pdef_info_array + defnrToPCIndex[index];

          deformed = 1;

          if (bone && bone->flag & BONE_MULT_VG_ENV) {
            weight *= distfactor_to_bone(
                co, bone->arm_head, bone->arm_tail, bone->rad_head, bone->rad_tail, bone->dist);
          }
          pchan_bone_deform(pchan, pdef_info, weight, vec, dq, smat, co, &contrib);
        }
      }
      /* if there are vertexgroups but not groups with bones
       * (like for softbody groups) */
      if (deformed == 0 && use_envelope) {
        pdef_info = pdef_info_array;
        for (pchan = (bPoseChannel *)armOb->pose->chanbase.first; pchan; pchan = pchan->next, pdef_info++) {
          if (!(pchan->bone->flag & BONE_NO_DEFORM))
            contrib += dist_bone_deform(pchan, pdef_info, vec, dq, smat, co);
        }
      }
    }
    else if (use_envelope) {
      pdef_info = pdef_info_array;
      for (pchan = (bPoseChannel *)armOb->pose->chanbase.first; pchan; pchan = pchan->next, pdef_info++) {
        if (!(pchan->bone->flag & BONE_NO_DEFORM))
          contrib += dist_bone_deform(pchan, pdef_info, vec, dq, smat, co);
      }
    }

    /* actually should be EPSILON? weight values and contrib can be like 10e-39 small */
    if (contrib > 0.0001f) {
      if (use_quaternion) {
        normalize_dq(dq, contrib);

        if (armature_weight != 1.0f) {
          copy_v3_v3(dco, co);
          mul_v3m3_dq(dco, (defMats) ? summat : NULL, dq);
          sub_v3_v3(dco, co);
          mul_v3_fl(dco, armature_weight);
          add_v3_v3(co, dco);
        }
        else
          mul_v3m3_dq(co, (defMats) ? summat : NULL, dq);

        smat = summat;
      }
      else {
        mul_v3_fl(vec, armature_weight / contrib);
        add_v3_v3v3(co, vec, co);
      }

      if (defMats) {
        float pre[3][3], post[3][3], tmpmat[3][3];

        copy_m3_m4(pre, premat);
        copy_m3_m4(post, postmat);
        copy_m3_m3(tmpmat, defMats[i]);

        if (!use_quaternion) /* quaternion already is scale corrected */
          mul_m3_fl(smat, armature_weight / contrib);

        mul_m3_series(defMats[i], post, smat, pre, tmpmat);
      }
    }

    /* always, check above code */
    mul_m4_v3(postmat, co);

    /* interpolate with previous modifier position using weight group */
    if (prevCos) {
      float mw = 1.0f - prevco_weight;
      vertexCos[i][0] = prevco_weight * vertexCos[i][0] + mw * co[0];
      vertexCos[i][1] = prevco_weight * vertexCos[i][1] + mw * co[1];
      vertexCos[i][2] = prevco_weight * vertexCos[i][2] + mw * co[2];
    }
  }

  if (dualquats)
    MEM_freeN(dualquats);
  if (defnrToPC)
    MEM_freeN(defnrToPC);
  if (defnrToPCIndex)
    MEM_freeN(defnrToPCIndex);

  /* free B_bone matrices */
  pdef_info = pdef_info_array;
  for (pchan = (bPoseChannel *)armOb->pose->chanbase.first; pchan;
       pchan = pchan->next, pdef_info++) {
    if (pdef_info->b_bone_mats)
      MEM_freeN(pdef_info->b_bone_mats);
    if (pdef_info->b_bone_dual_quats)
      MEM_freeN(pdef_info->b_bone_dual_quats);
  }

  MEM_freeN(pdef_info_array);

  // restore matrix
  copy_m4_m4(target->obmat, obmat);
}

void BL_ArmatureObject::LoadConstraints(BL_BlenderSceneConverter *converter)
{
  // first delete any existing constraint (should not have any)
  m_controlledConstraints->ReleaseAndRemoveAll();

  // list all the constraint and convert them to BL_ArmatureConstraint
  // get the persistent pose structure

  // and locate the constraint
  for (bPoseChannel *pchan = (bPoseChannel *)m_objArma->pose->chanbase.first; pchan;
       pchan = pchan->next) {
    for (bConstraint *pcon = (bConstraint *)pchan->constraints.first; pcon; pcon = pcon->next) {
      if (pcon->flag & CONSTRAINT_DISABLE) {
        continue;
      }
      // which constraint should we support?
      switch (pcon->type) {
        case CONSTRAINT_TYPE_TRACKTO:
        case CONSTRAINT_TYPE_DAMPTRACK:
        case CONSTRAINT_TYPE_KINEMATIC:
        case CONSTRAINT_TYPE_ROTLIKE:
        case CONSTRAINT_TYPE_LOCLIKE:
        case CONSTRAINT_TYPE_MINMAX:
        case CONSTRAINT_TYPE_SIZELIKE:
        case CONSTRAINT_TYPE_LOCKTRACK:
        case CONSTRAINT_TYPE_STRETCHTO:
        case CONSTRAINT_TYPE_CLAMPTO:
        case CONSTRAINT_TYPE_TRANSFORM:
        case CONSTRAINT_TYPE_DISTLIMIT:
        case CONSTRAINT_TYPE_TRANSLIKE: {
          const bConstraintTypeInfo *cti = BKE_constraint_typeinfo_get(pcon);
          KX_GameObject *gametarget = nullptr;
          KX_GameObject *gamesubtarget = nullptr;
          if (cti && cti->get_constraint_targets) {
            ListBase listb = {nullptr, nullptr};
            cti->get_constraint_targets(pcon, &listb);
            if (listb.first) {
              bConstraintTarget *target = (bConstraintTarget *)listb.first;
              if (target->tar && target->tar != m_objArma) {
                // only remember external objects, self target is handled automatically
                gametarget = converter->FindGameObject(target->tar);
              }
              if (target->next != nullptr) {
                // secondary target
                target = target->next;
                if (target->tar && target->tar != m_objArma) {
                  // only track external object
                  gamesubtarget = converter->FindGameObject(target->tar);
                }
              }
            }
            if (cti->flush_constraint_targets) {
              cti->flush_constraint_targets(pcon, &listb, 1);
            }
          }
          BL_ArmatureConstraint *constraint = new BL_ArmatureConstraint(
              this, pchan, pcon, gametarget, gamesubtarget);
          m_controlledConstraints->Add(constraint);
        }
      }
    }
  }

  // If we have constraints, make sure we get treated as an "animated" object
  if (m_controlledConstraints->GetCount() > 0) {
    GetActionManager();
  }
}

size_t BL_ArmatureObject::GetConstraintNumber() const
{
  return m_controlledConstraints->GetCount();
}

BL_ArmatureConstraint *BL_ArmatureObject::GetConstraint(const std::string &posechannel,
                                                        const std::string &constraintname)
{
  return m_controlledConstraints->FindIf(
      [&posechannel, &constraintname](BL_ArmatureConstraint *constraint) {
        return constraint->Match(posechannel, constraintname);
      });
}

BL_ArmatureConstraint *BL_ArmatureObject::GetConstraint(const std::string &posechannelconstraint)
{
  return static_cast<BL_ArmatureConstraint *>(
      m_controlledConstraints->FindValue(posechannelconstraint));
}

BL_ArmatureConstraint *BL_ArmatureObject::GetConstraint(int index)
{
  return static_cast<BL_ArmatureConstraint *>(m_controlledConstraints->GetValue(index));
}

/* this function is called to populate the m_poseChannels list */
void BL_ArmatureObject::LoadChannels()
{
  if (m_poseChannels->GetCount() == 0) {
    for (bPoseChannel *pchan = (bPoseChannel *)m_objArma->pose->chanbase.first; pchan;
         pchan = (bPoseChannel *)pchan->next) {
      BL_ArmatureChannel *proxy = new BL_ArmatureChannel(this, pchan);
      m_poseChannels->Add(proxy);
    }
  }
}

size_t BL_ArmatureObject::GetChannelNumber() const
{
  return m_poseChannels->GetCount();
}

BL_ArmatureChannel *BL_ArmatureObject::GetChannel(bPoseChannel *pchan)
{
  LoadChannels();
  return m_poseChannels->FindIf(
      [&pchan](BL_ArmatureChannel *channel) { return channel->m_posechannel == pchan; });
}

BL_ArmatureChannel *BL_ArmatureObject::GetChannel(const std::string &str)
{
  LoadChannels();
  return static_cast<BL_ArmatureChannel *>(m_poseChannels->FindValue(str));
}

BL_ArmatureChannel *BL_ArmatureObject::GetChannel(int index)
{
  LoadChannels();
  if (index < 0 || index >= m_poseChannels->GetCount()) {
    return nullptr;
  }
  return static_cast<BL_ArmatureChannel *>(m_poseChannels->GetValue(index));
}

EXP_Value *BL_ArmatureObject::GetReplica()
{
  BL_ArmatureObject *replica = new BL_ArmatureObject(*this);
  replica->ProcessReplica();
  return replica;
}

void BL_ArmatureObject::ProcessReplica()
{
  KX_GameObject::ProcessReplica();

  // Replicate each constraints.
  m_controlledConstraints = static_cast<EXP_ListValue<BL_ArmatureConstraint> *>(
      m_controlledConstraints->GetReplica());
  // Share pose channels.
  m_poseChannels->AddRef();

  m_objArma = m_pBlenderObject;
}

int BL_ArmatureObject::GetGameObjectType() const
{
  return OBJ_ARMATURE;
}

void BL_ArmatureObject::ReParentLogic()
{
  for (BL_ArmatureConstraint *constraint : m_controlledConstraints) {
    constraint->ReParent(this);
  }
  KX_GameObject::ReParentLogic();
}

void BL_ArmatureObject::Relink(std::map<SCA_IObject *, SCA_IObject *> &obj_map)
{
  for (BL_ArmatureConstraint *constraint : m_controlledConstraints) {
    constraint->Relink(obj_map);
  }
  KX_GameObject::Relink(obj_map);
}

bool BL_ArmatureObject::UnlinkObject(SCA_IObject *clientobj)
{
  // clientobj is being deleted, make sure we don't hold any reference to it
  bool res = false;
  for (BL_ArmatureConstraint *constraint : m_controlledConstraints) {
    res |= constraint->UnlinkObject(clientobj);
  }
  return res;
}

void BL_ArmatureObject::ApplyPose()
{
  if (m_lastapplyframe != m_lastframe) {
    // update the constraint if any, first put them all off so that only the active ones will be
    // updated
    for (BL_ArmatureConstraint *constraint : m_controlledConstraints) {
      constraint->UpdateTarget();
    }
    // update ourself
    UpdateBlenderObjectMatrix(m_objArma);
    bContext *C = KX_GetActiveEngine()->GetContext();
    Depsgraph *depsgraph = CTX_data_depsgraph_on_load(C);
    BKE_pose_where_is(depsgraph, m_scene, m_objArma);
    // restore ourself
    memcpy(m_objArma->obmat, m_obmat, sizeof(m_obmat));
    m_lastapplyframe = m_lastframe;
  }
}

void BL_ArmatureObject::SetPoseByAction(bAction *action, AnimationEvalContext *evalCtx)
{
  PointerRNA ptrrna;
  RNA_id_pointer_create(&m_objArma->id, &ptrrna);

  animsys_evaluate_action(&ptrrna, action, evalCtx, false);
}

void BL_ArmatureObject::BlendInPose(bPose *blend_pose, float weight, short mode)
{
  game_blend_poses(m_objArma->pose, blend_pose, weight, mode);
}

bool BL_ArmatureObject::UpdateTimestep(double curtime)
{
  if (curtime != m_lastframe) {
    /* Compute the timestep for the underlying IK algorithm,
     * in the GE, we use ctime to store the timestep.
     */
    m_objArma->pose->ctime = (float)(curtime - m_lastframe);
    m_lastframe = curtime;
  }

  return false;
}

Object *BL_ArmatureObject::GetArmatureObject()
{
  return m_objArma;
}
Object *BL_ArmatureObject::GetOrigArmatureObject()
{
  return m_origObjArma;
}

void BL_ArmatureObject::GetPose(bPose **pose) const
{
  /* If the caller supplies a null pose, create a new one. */
  /* Otherwise, copy the armature's pose channels into the caller-supplied pose */

  if (!*pose) {
    /* probably not to good of an idea to
     * duplicate everything, but it clears up
     * a crash and memory leakage when
     * &BL_ActionActuator::m_pose is freed
     */
    BKE_pose_copy_data(pose, m_objArma->pose, 1);
  }
  else {
    if (*pose == m_objArma->pose) {
      // no need to copy if the pointers are the same
      return;
    }

    extract_pose_from_pose(*pose, m_objArma->pose);
  }
}

bPose *BL_ArmatureObject::GetPose() const
{
  return m_objArma->pose;
}

double BL_ArmatureObject::GetLastFrame()
{
  return m_lastframe;
}

bool BL_ArmatureObject::GetBoneMatrix(Bone *bone, MT_Matrix4x4 &matrix)
{
  ApplyPose();
  bPoseChannel *pchan = BKE_pose_channel_find_name(m_objArma->pose, bone->name);
  if (pchan) {
    matrix.setValue(&pchan->pose_mat[0][0]);
  }

  return (pchan != nullptr);
}

bool BL_ArmatureObject::GetDrawDebug() const
{
  return m_drawDebug;
}

void BL_ArmatureObject::DrawDebug(RAS_DebugDraw &debugDraw)
{
  const MT_Vector3 &scale = NodeGetWorldScaling();
  const MT_Matrix3x3 &rot = NodeGetWorldOrientation();
  const MT_Vector3 &pos = NodeGetWorldPosition();

  for (bPoseChannel *pchan = (bPoseChannel *)m_objArma->pose->chanbase.first; pchan;
       pchan = pchan->next) {
    MT_Vector3 head = rot * (MT_Vector3(pchan->pose_head) * scale) + pos;
    MT_Vector3 tail = rot * (MT_Vector3(pchan->pose_tail) * scale) + pos;
    debugDraw.DrawLine(tail, head, MT_Vector4(1.0f, 0.0f, 0.0f, 1.0f));
  }
  m_drawDebug = false;
}

float BL_ArmatureObject::GetBoneLength(Bone *bone) const
{
  return (float)(MT_Vector3(bone->head) - MT_Vector3(bone->tail)).length();
}

#ifdef WITH_PYTHON

// PYTHON

PyTypeObject BL_ArmatureObject::Type = {PyVarObject_HEAD_INIT(nullptr, 0) "BL_ArmatureObject",
                                        sizeof(EXP_PyObjectPlus_Proxy),
                                        0,
                                        py_base_dealloc,
                                        0,
                                        0,
                                        0,
                                        0,
                                        py_base_repr,
                                        0,
                                        &KX_GameObject::Sequence,
                                        &KX_GameObject::Mapping,
                                        0,
                                        0,
                                        0,
                                        nullptr,
                                        nullptr,
                                        0,
                                        Py_TPFLAGS_DEFAULT | Py_TPFLAGS_BASETYPE,
                                        0,
                                        0,
                                        0,
                                        0,
                                        0,
                                        0,
                                        0,
                                        Methods,
                                        0,
                                        0,
                                        &KX_GameObject::Type,
                                        0,
                                        0,
                                        0,
                                        0,
                                        0,
                                        0,
                                        py_base_new};

PyMethodDef BL_ArmatureObject::Methods[] = {
    EXP_PYMETHODTABLE_NOARGS(BL_ArmatureObject, update),
    EXP_PYMETHODTABLE_NOARGS(BL_ArmatureObject, draw),
    {nullptr, nullptr}  // Sentinel
};

PyAttributeDef BL_ArmatureObject::Attributes[] = {

    EXP_PYATTRIBUTE_RO_FUNCTION("constraints", BL_ArmatureObject, pyattr_get_constraints),
    EXP_PYATTRIBUTE_RO_FUNCTION("channels", BL_ArmatureObject, pyattr_get_channels),
    EXP_PYATTRIBUTE_NULL  // Sentinel
};

PyObject *BL_ArmatureObject::pyattr_get_constraints(EXP_PyObjectPlus *self_v,
                                                    const EXP_PYATTRIBUTE_DEF *attrdef)
{
  BL_ArmatureObject *self = static_cast<BL_ArmatureObject *>(self_v);
  return self->m_controlledConstraints->GetProxy();
}

PyObject *BL_ArmatureObject::pyattr_get_channels(EXP_PyObjectPlus *self_v,
                                                 const EXP_PYATTRIBUTE_DEF *attrdef)
{
  BL_ArmatureObject *self = static_cast<BL_ArmatureObject *>(self_v);
  self->LoadChannels();  // make sure we have the channels
  return self->m_poseChannels->GetProxy();
}

EXP_PYMETHODDEF_DOC_NOARGS(
    BL_ArmatureObject,
    update,
    "update()\n"
    "Make sure that the armature will be updated on next graphic frame.\n"
    "This is automatically done if a KX_ArmatureActuator with mode run is active\n"
    "or if an action is playing. This function is useful in other cases.\n")
{
  UpdateTimestep(KX_GetActiveEngine()->GetFrameTime());
  Py_RETURN_NONE;
}

EXP_PYMETHODDEF_DOC_NOARGS(BL_ArmatureObject, draw, "Draw Debug Armature")
{
  /* Armature bones are updated later, so we only set to true a flag
   * to request a debug draw later in ApplyPose after updating bones. */
  m_drawDebug = true;
  Py_RETURN_NONE;
}

#endif  // WITH_PYTHON
