#include <extern.hpp>
#include <gui/Main_screen/MainView.hpp>
#include <images/BitmapDatabase.hpp>

#include "main.h"

MainView::MainView() {

}


void MainView::setupScreen() {


	__background.setPosition(0, 0, LCD_RES_H, LCD_RES_V);
	__background.invalidate();

//    background.setPosition(-3, 1, LCD_RES_H, LCD_RES_V);
//	container_0.invalidate();

	MainViewBase::setupScreen();

	//Bitmap::cache(BITMAP_RACE_DASH_FULL_ID);

	container_0.initContainer(Dash_Settings.SCREEN_CONTAINERS[0]);
	container_0.invalidate();

	container_1.initContainer(Dash_Settings.SCREEN_CONTAINERS[1]);
	container_1.invalidate();

	container_2.initContainer(Dash_Settings.SCREEN_CONTAINERS[2]);
	container_2.invalidate();

	container_3.initContainer(Dash_Settings.SCREEN_CONTAINERS[3]);
	container_3.invalidate();

	container_4.initContainer(Dash_Settings.SCREEN_CONTAINERS[4]);
	container_4.invalidate();

	container_5.initContainer(Dash_Settings.SCREEN_CONTAINERS[5]);
	container_5.invalidate();

	container_6.initContainer(Dash_Settings.SCREEN_CONTAINERS[6]);
	container_6.invalidate();

	container_7.initContainer(Dash_Settings.SCREEN_CONTAINERS[7]);
	container_7.invalidate();


	message_container_0.initContainer(Dash_Settings.SCREEN_MESSAGE_CONTAINERS[0]);
	container_0.invalidate();

}

void MainView::tearDownScreen() {
	MainViewBase::tearDownScreen();
    Bitmap::clearCache();
}

void MainView::handleTickEvent() {

	if (Dash_Settings.SCREEN_FIELDS_CHANGED == true) {
		setupScreen();
		Dash_Settings.SCREEN_FIELDS_CHANGED = false;
	}

	container_0.updateContainer(Dash_Settings.SCREEN_CONTAINERS[0]);
	container_0.invalidate();

	container_1.updateContainer(Dash_Settings.SCREEN_CONTAINERS[1]);
	container_1.invalidate();

	container_2.updateContainer(Dash_Settings.SCREEN_CONTAINERS[2]);
	container_2.invalidate();

	container_3.updateContainer(Dash_Settings.SCREEN_CONTAINERS[3]);
	container_3.invalidate();

	container_4.updateContainer(Dash_Settings.SCREEN_CONTAINERS[4]);
	container_4.invalidate();

	container_5.updateContainer(Dash_Settings.SCREEN_CONTAINERS[5]);
	container_5.invalidate();

	container_6.updateContainer(Dash_Settings.SCREEN_CONTAINERS[6]);
	container_6.invalidate();

	container_7.updateContainer(Dash_Settings.SCREEN_CONTAINERS[7]);
	container_7.invalidate();


	if(Dash_Settings.SCREEN_MESSAGE_CONTAINERS[0].Enabled == 1)
	{
		message_container_0.updateContainer(Dash_Settings.SCREEN_MESSAGE_CONTAINERS[0]);
		message_container_0.setVisible(true);
		message_container_0.invalidate();
	} else {
		message_container_0.setVisible(false);
		message_container_0.invalidate();
	}

	indLeft.setVisible(Current_Status.IND_LEFT);
	indLeft.invalidate();

	indHigh.setVisible(Current_Status.IND_HIGH);
	indHigh.invalidate();

	indFuel.setVisible(Current_Status.IND_FUEL);
	indFuel.invalidate();

	indOil.setVisible(Current_Status.IND_OIL);
	indOil.invalidate();

	indBatt.setVisible(Current_Status.IND_BATT);
	indBatt.invalidate();

	indPark.setVisible(Current_Status.IND_PARK);
	indPark.invalidate();

	indDTC.setVisible(Current_Status.IND_DTC);
	indDTC.invalidate();

	indECT.setVisible(Current_Status.IND_ECT);
	indECT.invalidate();

	indLow.setVisible(Current_Status.IND_LOW);
	indLow.invalidate();

	indRight.setVisible(Current_Status.IND_RIGHT);
	indRight.invalidate();

	gauge1.updateValue(Current_Status.RPM, 0);

	Unicode::UnicodeChar buffer[10];
	Unicode::snprintfFloat(buffer, 10, "%.0f", Current_Status.RPM);
	Unicode::snprintf(rpmBuffer, 10, "%s", buffer);
	rpm.setVisible(true);
	rpm.invalidate();


	Unicode::snprintfFloat(buffer, 10, "%.0f", Current_Status.GEAR);
	Unicode::snprintf(gearBuffer, 10, "%s", buffer);
	gear.resizeToCurrentTextWithAlignment();
	gear.setVisible(true);
	gear.invalidate();

}
