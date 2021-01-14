/** \file gameengine/Expressions/BoolValue.cpp
 *  \ingroup expressions
 */

// BoolValue.cpp: implementation of the EXP_BoolValue class.
/*
 * Copyright (c) 1996-2000 Erwin Coumans <coockie@acm.org>
 *
 * Permission to use, copy, modify, distribute and sell this software
 * and its documentation for any purpose is hereby granted without fee,
 * provided that the above copyright notice appear in all copies and
 * that both that copyright notice and this permission notice appear
 * in supporting documentation.  Erwin Coumans makes no
 * representations about the suitability of this software for any
 * purpose.  It is provided "as is" without express or implied warranty.
 *
 */

#include "EXP_BoolValue.h"

#include "EXP_ErrorValue.h"
#include "EXP_StringValue.h"

const std::string EXP_BoolValue::sTrueString = "TRUE";
const std::string EXP_BoolValue::sFalseString = "FALSE";

EXP_BoolValue::EXP_BoolValue()
{
  trace("Bool constructor error");
}

EXP_BoolValue::EXP_BoolValue(bool inBool) : m_bool(inBool)
{
}

EXP_BoolValue::EXP_BoolValue(bool innie, const std::string &name) : m_bool(innie)
{
  SetName(name);
}

void EXP_BoolValue::SetValue(EXP_Value *newval)
{
  m_bool = (newval->GetNumber() != 0);
}

EXP_Value *EXP_BoolValue::Calc(VALUE_OPERATOR op, EXP_Value *val)
{
  switch (op) {
    case VALUE_POS_OPERATOR:
    case VALUE_NEG_OPERATOR: {
      return new EXP_ErrorValue(op2str(op) + GetText());
      break;
    }
    case VALUE_NOT_OPERATOR: {
      return new EXP_BoolValue(!m_bool);
      break;
    }
    default: {
      return val->CalcFinal(VALUE_BOOL_TYPE, op, this);
      break;
    }
  }
}

EXP_Value *EXP_BoolValue::CalcFinal(VALUE_DATA_TYPE dtype, VALUE_OPERATOR op, EXP_Value *val)
{
  EXP_Value *ret;

  switch (dtype) {
    case VALUE_EMPTY_TYPE:
    case VALUE_BOOL_TYPE: {
      switch (op) {
        case VALUE_AND_OPERATOR: {
          ret = new EXP_BoolValue(((EXP_BoolValue *)val)->GetBool() && m_bool);
          break;
        }
        case VALUE_OR_OPERATOR: {
          ret = new EXP_BoolValue(((EXP_BoolValue *)val)->GetBool() || m_bool);
          break;
        }
        case VALUE_EQL_OPERATOR: {
          ret = new EXP_BoolValue(((EXP_BoolValue *)val)->GetBool() == m_bool);
          break;
        }
        case VALUE_NEQ_OPERATOR: {
          ret = new EXP_BoolValue(((EXP_BoolValue *)val)->GetBool() != m_bool);
          break;
        }
        case VALUE_NOT_OPERATOR: {
          return new EXP_BoolValue(!m_bool);
          break;
        }
        default: {
          ret = new EXP_ErrorValue(val->GetText() + op2str(op) +
                                "[operator not allowed on booleans]");
          break;
        }
      }
      break;
    }
    case VALUE_STRING_TYPE: {
      switch (op) {
        case VALUE_ADD_OPERATOR: {
          ret = new EXP_StringValue(val->GetText() + GetText(), "");
          break;
        }
        default: {
          ret = new EXP_ErrorValue(val->GetText() + op2str(op) +
                                "[Only + allowed on boolean and string]");
          break;
        }
      }
      break;
    }
    default:
      ret = new EXP_ErrorValue("[type mismatch]" + op2str(op) + GetText());
  }

  return ret;
}

bool EXP_BoolValue::GetBool()
{
  return m_bool;
}

double EXP_BoolValue::GetNumber()
{
  return (double)m_bool;
}

int EXP_BoolValue::GetValueType()
{
  return VALUE_BOOL_TYPE;
}

std::string EXP_BoolValue::GetText()
{
  return m_bool ? sTrueString : sFalseString;
}

EXP_Value *EXP_BoolValue::GetReplica()
{
  EXP_BoolValue *replica = new EXP_BoolValue(*this);
  replica->ProcessReplica();

  return replica;
}

#ifdef WITH_PYTHON
PyObject *EXP_BoolValue::ConvertValueToPython()
{
  return PyBool_FromLong(m_bool != 0);
}
#endif  // WITH_PYTHON
