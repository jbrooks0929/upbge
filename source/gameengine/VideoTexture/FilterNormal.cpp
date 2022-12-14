/* SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright 2006 The Zdeno Ash Miklas. */

/** \file gameengine/VideoTexture/FilterNormal.cpp
 *  \ingroup bgevideotex
 */

#include "FilterNormal.h"

// implementation FilterNormal

// constructor
FilterNormal::FilterNormal(void) : m_colIdx(0)
{
  // set default depth
  setDepth(4);
}

// set color shift
void FilterNormal::setColor(unsigned short colIdx)
{
  // check validity of index
  if (colIdx < 3)
    // set color shift
    m_colIdx = colIdx;
}

// set depth
void FilterNormal::setDepth(float depth)
{
  m_depth = depth;
  m_depthScale = depth / depthScaleKoef;
}

// cast Filter pointer to FilterNormal
inline FilterNormal *getFilter(PyFilter *self)
{
  return static_cast<FilterNormal *>(self->m_filter);
}

// python methods and get/sets

// get index of color used to calculate normal
static PyObject *getColor(PyFilter *self, void *closure)
{
  return Py_BuildValue("H", getFilter(self)->getColor());
}

// set index of color used to calculate normal
static int setColor(PyFilter *self, PyObject *value, void *closure)
{
  // check validity of parameter
  if (value == nullptr || !PyLong_Check(value)) {
    PyErr_SetString(
        PyExc_TypeError,
        "filt.colorIdx = int: VideoTexture.FilterNormal, expected the value must be a int");
    return -1;
  }
  // set color index
  getFilter(self)->setColor((unsigned short)(PyLong_AsLong(value)));
  // success
  return 0;
}

// get depth
static PyObject *getDepth(PyFilter *self, void *closure)
{
  return Py_BuildValue("f", getFilter(self)->getDepth());
}

// set depth
static int setDepth(PyFilter *self, PyObject *value, void *closure)
{
  // check validity of parameter
  if (value) {
    float depth = (float)PyFloat_AsDouble(value);
    if ((depth == -1 && PyErr_Occurred()) == 0) /* no error converting to a float? */
    {
      // set depth
      getFilter(self)->setDepth(depth);
      // success
      return 0;
    }
  }

  PyErr_SetString(
      PyExc_TypeError,
      "filt.depth = float: VideoTexture.FilterNormal, expected the value must be a float");
  return -1;
}

// attributes structure
static PyGetSetDef filterNormalGetSets[] = {
    {(char *)"colorIdx",
     (getter)getColor,
     (setter)setColor,
     (char *)"index of color used to calculate normal (0 - red, 1 - green, 2 - blue)",
     nullptr},
    {(char *)"depth", (getter)getDepth, (setter)setDepth, (char *)"depth of relief", nullptr},
    // attributes from FilterBase class
    {(char *)"previous",
     (getter)Filter_getPrevious,
     (setter)Filter_setPrevious,
     (char *)"previous pixel filter",
     nullptr},
    {nullptr}};

// define python type
PyTypeObject FilterNormalType = {
    PyVarObject_HEAD_INIT(nullptr, 0) "VideoTexture.FilterNormal", /*tp_name*/
    sizeof(PyFilter),                                              /*tp_basicsize*/
    0,                                                             /*tp_itemsize*/
    (destructor)Filter_dealloc,                                    /*tp_dealloc*/
    0,                                                             /*tp_print*/
    0,                                                             /*tp_getattr*/
    0,                                                             /*tp_setattr*/
    0,                                                             /*tp_compare*/
    0,                                                             /*tp_repr*/
    0,                                                             /*tp_as_number*/
    0,                                                             /*tp_as_sequence*/
    0,                                                             /*tp_as_mapping*/
    0,                                                             /*tp_hash */
    0,                                                             /*tp_call*/
    0,                                                             /*tp_str*/
    0,                                                             /*tp_getattro*/
    0,                                                             /*tp_setattro*/
    0,                                                             /*tp_as_buffer*/
    Py_TPFLAGS_DEFAULT,                                            /*tp_flags*/
    "Filter for Blue Screen objects",                              /* tp_doc */
    0,                                                             /* tp_traverse */
    0,                                                             /* tp_clear */
    0,                                                             /* tp_richcompare */
    0,                                                             /* tp_weaklistoffset */
    0,                                                             /* tp_iter */
    0,                                                             /* tp_iternext */
    nullptr,                                                       /* tp_methods */
    0,                                                             /* tp_members */
    filterNormalGetSets,                                           /* tp_getset */
    0,                                                             /* tp_base */
    0,                                                             /* tp_dict */
    0,                                                             /* tp_descr_get */
    0,                                                             /* tp_descr_set */
    0,                                                             /* tp_dictoffset */
    (initproc)Filter_init<FilterNormal>,                           /* tp_init */
    0,                                                             /* tp_alloc */
    Filter_allocNew,                                               /* tp_new */
};
