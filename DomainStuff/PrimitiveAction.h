#ifndef PRIMITIVE_ACTION_H
#define PRIMITIVE_ACTION_H

#include "Action.h"

class PrimitiveAction : public Action {
public:
    virtual ~PrimitiveAction() = default;

    virtual bool operator==(const PrimitiveAction& other) const = 0;
    virtual bool operator<(const PrimitiveAction& other) const = 0;

};

#endif // PRIMITIVE_ACTION_H
