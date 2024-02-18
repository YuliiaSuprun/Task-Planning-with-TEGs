#ifndef ACTION_H
#define ACTION_H

#include <vector>

class Action {
public:
    virtual ~Action() = default;
    virtual bool isSkillAction() const { return false; }
};

#endif // ACTION_H