#ifndef MESSAGE_CONTAINER_HPP
#define MESSAGE_CONTAINER_HPP

#include <extern.hpp>
#include <gui_generated/containers/message_containerBase.hpp>
#include <touchgfx/Color.hpp>

#include "main.h"

class message_container : public message_containerBase
{
public:
    message_container();
    virtual ~message_container() {}

    virtual void initialize();

    virtual void initContainer(MESSAGE_CONTAINERS message);
    virtual void updateContainer(MESSAGE_CONTAINERS message);
protected:
};

#endif // MESSAGE_CONTAINER_HPP
