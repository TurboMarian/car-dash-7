#include <gui/containers/value_container.hpp>

value_container::value_container()
{

}

void value_container::initialize()
{
    value_containerBase::initialize();
}

void value_container::initContainer(CONTAINER channel) {
	Unicode::UnicodeChar buffer[32];

	container_background.setColor(
			touchgfx::Color::getColorFromRGB(channel.Background_Color.red,
					channel.Background_Color.green, channel.Background_Color.blue));
	container_background.setVisible(false);
	container_background.invalidate();

	container_value_L.setVisible(false);
	container_value_R.setVisible(false);
	container_value_C.setVisible(false);

	container_label_L.setVisible(false);
	container_label_R.setVisible(false);
	container_label_C.setVisible(false);

	container_unit_L.setVisible(false);
	container_unit_R.setVisible(false);
	container_unit_C.setVisible(false);


	switch (channel.Value.Alignment) {
		case ALIGN_LEFT:
			container_value_L.setVisible(true);
			container_value_L.setXY(channel.Value.X, channel.Value.Y);
			container_value_L.setWidthHeight(channel.Value.Width,
					channel.Value.Height);
			container_value_L.resizeToCurrentTextWithAlignment();
			container_value_L.invalidate();
			break;
		case ALIGN_RIGHT:
			container_value_R.setVisible(true);
			container_value_R.setXY(channel.Value.X, channel.Value.Y);
			container_value_R.setWidthHeight(channel.Value.Width,
					channel.Value.Height);
			container_value_R.resizeToCurrentTextWithAlignment();
			container_value_R.invalidate();
			break;
		default:
			container_value_C.setVisible(true);
			container_value_C.setXY(channel.Value.X, channel.Value.Y);
			container_value_C.setWidthHeight(channel.Value.Width,
					channel.Value.Height);
			container_value_C.resizeToCurrentTextWithAlignment();
			container_value_C.invalidate();
			break;
	}

	switch (channel.Value.Alignment) {
		case ALIGN_LEFT:
			container_label_L.setVisible(true);
			Unicode::strncpy(buffer, channel.Label.Text, 32);
			Unicode::snprintf(container_label_LBuffer, 32, "%s", buffer);

			container_label_L.setXY(channel.Label.X, channel.Label.Y);
			container_label_L.setWidthHeight(channel.Label.Width,
					channel.Label.Height);
			container_label_L.resizeToCurrentTextWithAlignment();
			container_label_L.invalidate();
			break;
		case ALIGN_RIGHT:
			container_label_R.setVisible(true);
			Unicode::strncpy(buffer, channel.Label.Text, 32);
			Unicode::snprintf(container_label_RBuffer, 32, "%s", buffer);

			container_label_R.setXY(channel.Label.X, channel.Label.Y);
			container_label_R.setWidthHeight(channel.Label.Width,
					channel.Label.Height);
			container_label_R.resizeToCurrentTextWithAlignment();
			container_label_R.invalidate();
			break;
		default:
			container_label_C.setVisible(true);
			Unicode::strncpy(buffer, channel.Label.Text, 32);
			Unicode::snprintf(container_label_CBuffer, 32, "%s", buffer);

			container_label_C.setXY(channel.Label.X, channel.Label.Y);
			container_label_C.setWidthHeight(channel.Label.Width,
					channel.Label.Height);
			container_label_C.resizeToCurrentTextWithAlignment();
			container_label_C.invalidate();
			break;
	}

	switch (channel.Value.Alignment) {
		case ALIGN_LEFT:
			container_unit_L.setVisible(true);
			Unicode::strncpy(buffer, channel.Unit.Text, 32);
			Unicode::snprintf(container_unit_LBuffer, 32, "%s", buffer);

			container_unit_L.setXY(channel.Unit.X, channel.Unit.Y);
			container_unit_L.setWidthHeight(channel.Unit.Width,
					channel.Unit.Height);
			container_unit_L.resizeToCurrentTextWithAlignment();
			container_unit_L.invalidate();
			break;
		case ALIGN_RIGHT:
			container_unit_R.setVisible(true);
			Unicode::strncpy(buffer, channel.Unit.Text, 32);
			Unicode::snprintf(container_unit_RBuffer, 32, "%s", buffer);

			container_unit_R.setXY(channel.Unit.X, channel.Unit.Y);
			container_unit_R.setWidthHeight(channel.Unit.Width,
					channel.Unit.Height);
			container_unit_R.resizeToCurrentTextWithAlignment();
			container_unit_R.invalidate();
			break;
		default:
			container_unit_C.setVisible(true);
			Unicode::strncpy(buffer, channel.Unit.Text, 32);
			Unicode::snprintf(container_unit_CBuffer, 32, "%s", buffer);

			container_unit_C.setXY(channel.Unit.X, channel.Unit.Y);
			container_unit_C.setWidthHeight(channel.Unit.Width,
					channel.Unit.Height);
			container_unit_C.resizeToCurrentTextWithAlignment();
			container_unit_C.invalidate();
			break;
	}
}

void value_container::updateContainer(ITEM type, CONTAINER channel) {
	Unicode::UnicodeChar buffer[32];

	container_background.setColor(
			touchgfx::Color::getColorFromRGB(channel.Background_Color.red,
					channel.Background_Color.green, channel.Background_Color.blue));
	container_background.setVisible(true);
	container_background.invalidate();

	switch (type) {
		case LABEL:
			container_label_L.invalidate();
			Unicode::strncpy(buffer, channel.Label.Text, 32);
			Unicode::snprintf(container_label_LBuffer, 32, "%s", buffer);
			container_label_L.setColor(touchgfx::Color::getColorFromRGB(channel.Label.Text_Color.red,
					channel.Label.Text_Color.green, channel.Label.Text_Color.blue));
			container_label_L.resizeToCurrentTextWithAlignment();
			container_label_L.invalidate();
			break;
		case UNIT:
			container_unit_L.invalidate();
			Unicode::strncpy(buffer, channel.Unit.Text, 32);
			Unicode::snprintf(container_unit_LBuffer, 32, "%s", buffer);
			container_unit_L.resizeToCurrentTextWithAlignment();
			container_unit_L.invalidate();
			break;
		case BACKGROUND:
			break;
		case VALUE:
			float value =
					((channel.Data.Value * 1.0) / (channel.Data.Divider * 1.0))
							+ (channel.Data.Adder * 1.0);

			switch (channel.Data.Decimal) {
			case 0:
				Unicode::snprintfFloat(buffer, 32, "%.0f", value);
				break;
			case 1:
				Unicode::snprintfFloat(buffer, 32, "%.1f", value);
				break;
			case 2:
				Unicode::snprintfFloat(buffer, 32, "%.2f", value);
				break;
			}

			switch (channel.Value.Alignment) {
				case ALIGN_LEFT:
					container_value_L.invalidate();
					Unicode::snprintf(container_value_LBuffer, 32, "%s", buffer);
					container_value_L.setColor(touchgfx::Color::getColorFromRGB(channel.Value.Text_Color.red,
							channel.Value.Text_Color.green, channel.Value.Text_Color.blue));
					container_value_L.resizeToCurrentTextWithAlignment();
					container_value_L.invalidate();
					break;
				case ALIGN_RIGHT:
					container_value_R.invalidate();
					Unicode::snprintf(container_value_RBuffer, 32, "%s", buffer);
					container_value_R.setColor(touchgfx::Color::getColorFromRGB(channel.Value.Text_Color.red,
							channel.Value.Text_Color.green, channel.Value.Text_Color.blue));
					container_value_R.resizeToCurrentTextWithAlignment();
					container_value_R.invalidate();
					break;
				default:
					container_value_C.invalidate();
					Unicode::snprintf(container_value_CBuffer, 32, "%s", buffer);
					container_value_C.setColor(touchgfx::Color::getColorFromRGB(channel.Value.Text_Color.red,
							channel.Value.Text_Color.green, channel.Value.Text_Color.blue));
					container_value_C.resizeToCurrentTextWithAlignment();
					container_value_C.invalidate();
					break;
			}

			break;
	}
}

