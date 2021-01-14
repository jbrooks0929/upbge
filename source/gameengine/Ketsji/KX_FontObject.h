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

/** \file KX_FontObject.h
 *  \ingroup ketsji
 */

#pragma once


#include "KX_GameObject.h"

class KX_FontObject : public KX_GameObject {
 public:
  Py_Header KX_FontObject(void *sgReplicationInfo,
                          SG_Callbacks callbacks,
                          RAS_Rasterizer *rasterizer,
                          Object *ob);

  virtual ~KX_FontObject();

  /**
   * Inherited from EXP_Value -- return a new copy of this
   * instance allocated on the heap. Ownership of the new
   * object belongs with the caller.
   */
  virtual EXP_Value *GetReplica();
  virtual void ProcessReplica();
  virtual int GetGameObjectType() const
  {
    return OBJ_TEXT;
  }

  void UpdateCurveText(std::string text);  // eevee

  // Update text and bounding box.
  void SetText(const std::string &text);
  /// Update text from property.
  void UpdateTextFromProperty();

 protected:
  std::string m_text;
  std::vector<std::string> m_texts;
  Object *m_object;

  std::string m_backupText;  // eevee
  /// needed for drawing routine
  class RAS_Rasterizer *m_rasterizer;
};

