#ifndef BOOTVIEW_HPP
#define BOOTVIEW_HPP

#include <gui_generated/boot_screen/BootViewBase.hpp>
#include <gui/boot_screen/BootPresenter.hpp>

class BootView : public BootViewBase
{
public:
    BootView();
    virtual ~BootView() {}
    virtual void setupScreen();
    virtual void tearDownScreen();
    virtual void handleTickEvent();
protected:
};

#endif // BOOTVIEW_HPP
