#include <extern.hpp>
#include <gui/boot_screen/BootView.hpp>
#include "main.h"

BootView::BootView()
{

}

void BootView::setupScreen()
{
    BootViewBase::setupScreen();
}

void BootView::tearDownScreen()
{
    BootViewBase::tearDownScreen();
}


void BootView::handleTickEvent() {

	if(Dash_Settings.ACTIVE_SCREEN == 1)
	{
		static_cast<FrontendApplication*>(Application::getInstance())->gotoMainScreenNoTransition();
	}

}

