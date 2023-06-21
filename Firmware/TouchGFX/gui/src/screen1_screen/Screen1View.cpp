#include <gui/screen1_screen/Screen1View.hpp>

#include "main.h"
#include "extern.h"

Screen1View::Screen1View() {

}

void Screen1View::setupScreen() {
	Screen1ViewBase::setupScreen();


	container_0.initContainer(Current_Status.SCREEN_CONTAINERS[0]);
	container_0.invalidate();

	container_1.initContainer(Current_Status.SCREEN_CONTAINERS[1]);
	container_1.invalidate();

	container_2.initContainer(Current_Status.SCREEN_CONTAINERS[2]);
	container_2.invalidate();

	container_3.initContainer(Current_Status.SCREEN_CONTAINERS[3]);
	container_3.invalidate();

	container_4.initContainer(Current_Status.SCREEN_CONTAINERS[4]);
	container_4.invalidate();

	container_5.initContainer(Current_Status.SCREEN_CONTAINERS[5]);
	container_5.invalidate();

	container_6.initContainer(Current_Status.SCREEN_CONTAINERS[6]);
	container_6.invalidate();

	container_7.initContainer(Current_Status.SCREEN_CONTAINERS[7]);
	container_7.invalidate();


	message_container_0.initContainer(Current_Status.SCREEN_MESSAGE_CONTAINERS[0]);
	container_0.invalidate();
}

void Screen1View::tearDownScreen() {
	Screen1ViewBase::tearDownScreen();
}

void Screen1View::handleTickEvent() {
	Unicode::UnicodeChar buffer[16];
	if (Current_Status.SCREEN_FIELDS_CHANGED == true) {
		setupScreen();
		Current_Status.SCREEN_FIELDS_CHANGED = false;
	}

	container_0.updateContainer(VALUE, Current_Status.SCREEN_CONTAINERS[0]);
	container_0.invalidate();

	container_1.updateContainer(VALUE, Current_Status.SCREEN_CONTAINERS[1]);
	container_1.invalidate();

	container_2.updateContainer(VALUE, Current_Status.SCREEN_CONTAINERS[2]);
	container_2.invalidate();

	container_3.updateContainer(VALUE, Current_Status.SCREEN_CONTAINERS[3]);
	container_3.invalidate();

	container_4.updateContainer(VALUE, Current_Status.SCREEN_CONTAINERS[4]);
	container_4.invalidate();

	container_5.updateContainer(VALUE, Current_Status.SCREEN_CONTAINERS[5]);
	container_5.invalidate();

	container_6.updateContainer(VALUE, Current_Status.SCREEN_CONTAINERS[6]);
	container_6.invalidate();

	container_7.updateContainer(VALUE, Current_Status.SCREEN_CONTAINERS[7]);
	container_7.invalidate();


	if(Current_Status.SCREEN_MESSAGE_CONTAINERS[0].Enabled == 1)
	{
		message_container_0.updateContainer(Current_Status.SCREEN_MESSAGE_CONTAINERS[0]);
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

}
