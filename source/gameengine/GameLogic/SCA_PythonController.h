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

/** \file SCA_PythonController.h
 *  \ingroup gamelogic
 *  \brief Execute Python scripts
 */

#pragma once


#include <vector>

#include "EXP_BoolValue.h"
#include "SCA_IController.h"
#include "SCA_LogicManager.h"

class SCA_IObject;
class SCA_PythonController : public SCA_IController {
  Py_Header
#ifdef WITH_PYTHON
      struct _object *m_bytecode; /* SCA_PYEXEC_SCRIPT only */
  PyObject *m_function;           /* SCA_PYEXEC_MODULE only */
#endif
  int m_function_argc;
  bool m_bModified;
  bool m_debug; /* use with SCA_PYEXEC_MODULE for reloading every logic run */
  int m_mode;

 protected:
  std::string m_scriptText;
  std::string m_scriptName;
#ifdef WITH_PYTHON
  PyObject *m_pythondictionary; /* for SCA_PYEXEC_SCRIPT only */
  PyObject *m_pythonfunction;   /* for SCA_PYEXEC_MODULE only */
#endif
  std::vector<class SCA_ISensor *> m_triggeredSensors;

 public:
  enum SCA_PyExecMode { SCA_PYEXEC_SCRIPT = 0, SCA_PYEXEC_MODULE, SCA_PYEXEC_MAX };

  static SCA_PythonController *m_sCurrentController;  // protected !!!

  // for debugging
  // virtual	EXP_Value*		AddRef();
  // virtual int			Release();  // Release a reference to this value (when reference count
  // reaches 0, the value is removed from the heap)

  SCA_PythonController(SCA_IObject *gameobj, int mode);
  virtual ~SCA_PythonController();

  virtual EXP_Value *GetReplica();
  virtual void Trigger(class SCA_LogicManager *logicmgr);

  void SetScriptText(const std::string &text);
  void SetScriptName(const std::string &name);
  void SetDebug(bool debug)
  {
    m_debug = debug;
  }
  void AddTriggeredSensor(class SCA_ISensor *sensor)
  {
    m_triggeredSensors.push_back(sensor);
  }
  bool IsTriggered(class SCA_ISensor *sensor);
  bool Compile();
  bool Import();
  void ErrorPrint(const char *error_msg);

#ifdef WITH_PYTHON
  static const char *sPyGetCurrentController__doc__;
  static PyObject *sPyGetCurrentController(PyObject *self);
  static const char *sPyAddActiveActuator__doc__;
  static PyObject *sPyAddActiveActuator(PyObject *self, PyObject *args);
  static SCA_IActuator *LinkedActuatorFromPy(PyObject *value);

  EXP_PYMETHOD_O(SCA_PythonController, Activate);
  EXP_PYMETHOD_O(SCA_PythonController, DeActivate);
  EXP_PYMETHOD_O(SCA_PythonController, SetScript);
  EXP_PYMETHOD_NOARGS(SCA_PythonController, GetScript);

  static PyObject *pyattr_get_script(EXP_PyObjectPlus *self_v, const EXP_PYATTRIBUTE_DEF *attrdef);
  static int pyattr_set_script(EXP_PyObjectPlus *self_v,
                               const EXP_PYATTRIBUTE_DEF *attrdef,
                               PyObject *value);
#endif
};

