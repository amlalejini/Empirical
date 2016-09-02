/*
  BodyOwner2D.h

  A base class that implements the necessary functions for body owners. 

*/
#include "Body2D.h"

#ifndef EMP_BODYOWNER_H
#define EMP_BODYOWNER_H

namespace emp {

template<typename BODY_TYPE>
class BodyOwner_Base {
protected:
  BODY_TYPE *body;
  bool has_body;

public:

  virtual ~BodyOwner_Base() {
    if (has_body) {
      delete body;
      DetachBody();
    }
  }

  virtual BODY_TYPE * GetBodyPtr() { emp_assert(has_body); return body; };
  virtual BODY_TYPE & GetBody() { emp_assert(has_body); return *body; };
  virtual const BODY_TYPE & GetConstBody() const { emp_assert(has_body); return *body; }

  virtual bool HasBody() const { return has_body; }
  virtual void AttachBody(BODY_TYPE * in_body) {
    body = in_body;
    has_body = true;
  }
  virtual void DetachBody() {
    body = nullptr;
    has_body = false;
  }
  virtual void Evaluate() {
    if (body->GetDestroyFlag()) {
      delete body;
      DetachBody();
    }
  }
};

}
#endif
