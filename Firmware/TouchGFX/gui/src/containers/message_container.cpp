#include <gui/containers/message_container.hpp>

message_container::message_container()
{

}

void message_container::initialize()
{
    message_containerBase::initialize();
}


void message_container::initContainer(MESSAGE_CONTAINERS message) {

	container_background.setColor(
			touchgfx::Color::getColorFromRGB(message.Background_Color.red,
					message.Background_Color.green, message.Background_Color.blue));
	container_background.setVisible(false);
	container_background.invalidate();

	container_value_L.setVisible(false);
	container_value_R.setVisible(false);
	container_value_C.setVisible(false);


	switch (message.Alignment) {
		case ALIGN_LEFT:
			container_value_L.setVisible(true);
			container_value_L.setXY(message.X, message.Y);
			container_value_L.setWidthHeight(message.Width, message.Height);
			container_value_L.setColor(touchgfx::Color::getColorFromRGB(message.Text_Color.red, message.Text_Color.green, message.Text_Color.blue));
			container_value_L.resizeToCurrentTextWithAlignment();
			break;
		case ALIGN_RIGHT:
			container_value_R.setVisible(true);
			container_value_R.setXY(message.X, message.Y);
			container_value_R.setWidthHeight(message.Width, message.Height);
			container_value_R.setColor(touchgfx::Color::getColorFromRGB(message.Text_Color.red, message.Text_Color.green, message.Text_Color.blue));
			container_value_R.resizeToCurrentTextWithAlignment();
			break;
		default:
			container_value_C.setVisible(true);
			container_value_C.setXY(message.X, message.Y);
			container_value_C.setWidthHeight(message.Width, message.Height);
			container_value_C.setColor(touchgfx::Color::getColorFromRGB(message.Text_Color.red, message.Text_Color.green, message.Text_Color.blue));
			container_value_C.resizeToCurrentTextWithAlignment();
			break;
	}

	container_value_L.invalidate();
	container_value_R.invalidate();
	container_value_C.invalidate();
}

void message_container::updateContainer(MESSAGE_CONTAINERS message) {
	Unicode::UnicodeChar buffer[256];

	container_background.setColor(
				touchgfx::Color::getColorFromRGB(message.Background_Color.red,
						message.Background_Color.green, message.Background_Color.blue));
	container_background.setVisible(true);
	container_background.invalidate();

	Unicode::strncpy(buffer, message.Text, 256);

	container_value_L.setVisible(false);
	container_value_R.setVisible(false);
	container_value_C.setVisible(false);

	switch (message.Alignment) {
		case ALIGN_LEFT:
			container_value_L.setVisible(true);
			Unicode::snprintf(container_value_LBuffer, 256, "%s", buffer);
			container_value_L.setColor(touchgfx::Color::getColorFromRGB(message.Text_Color.red,
					message.Text_Color.green, message.Text_Color.blue));
			container_value_L.resizeToCurrentTextWithAlignment();
			break;
		case ALIGN_RIGHT:
			container_value_R.setVisible(true);
			Unicode::snprintf(container_value_RBuffer, 256, "%s", buffer);
			container_value_R.setColor(touchgfx::Color::getColorFromRGB(message.Text_Color.red,
					message.Text_Color.green, message.Text_Color.blue));
			container_value_R.resizeToCurrentTextWithAlignment();
			break;
		default:
			container_value_C.setVisible(true);
			Unicode::snprintf(container_value_CBuffer, 256, "%s", buffer);
			container_value_C.setColor(touchgfx::Color::getColorFromRGB(message.Text_Color.red,
					message.Text_Color.green, message.Text_Color.blue));
			container_value_C.resizeToCurrentTextWithAlignment();
			break;
	}

	container_value_L.invalidate();
	container_value_R.invalidate();
	container_value_C.invalidate();
}
