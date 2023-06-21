#ifndef VALUE_CONTAINER_HPP
#define VALUE_CONTAINER_HPP

#include <gui_generated/containers/value_containerBase.hpp>
#include <touchgfx/Color.hpp>

#include "main.h"
#include "extern.h"

class value_container : public value_containerBase
{
public:
    value_container();
    virtual ~value_container() {}

    virtual void initialize();

    virtual void initContainer(CONTAINER channel);
    virtual void updateContainer(ITEM type, CONTAINER channel);
protected:
};

#endif // VALUE_CONTAINER_HPP
