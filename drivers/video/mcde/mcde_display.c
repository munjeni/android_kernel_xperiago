/*
 * Copyright (C) ST-Ericsson SA 2010
 *
 * ST-Ericsson MCDE display driver
 *
 * Author: Marcus Lorentzon <marcus.xm.lorentzon@stericsson.com>
 * for ST-Ericsson.
 *
 * License terms: GNU General Public License (GPL), version 2.
 */

#include <linux/kernel.h>
#include <linux/device.h>

#include <video/mcde_display.h>


static void mcde_display_get_native_resolution_default(
	struct mcde_display_device *ddev, u16 *x_res, u16 *y_res)
{
	/*
	 * If orientation is 90 or 270 then compensate the results
	 * Caller does not know about the orientation of the display
	 */
	if (ddev->orientation == MCDE_DISPLAY_ROT_90_CCW ||
				ddev->orientation == MCDE_DISPLAY_ROT_90_CW) {
		if (x_res)
			*x_res = ddev->native_y_res;
		if (y_res)
			*y_res = ddev->native_x_res;
	} else {
		if (x_res)
			*x_res = ddev->native_x_res;
		if (y_res)
			*y_res = ddev->native_y_res;

	}
}

static enum mcde_ovly_pix_fmt mcde_display_get_default_pixel_format_default(
	struct mcde_display_device *ddev)
{
	return ddev->default_pixel_format;
}

static void mcde_display_get_physical_size_default(
	struct mcde_display_device *ddev, u16 *width, u16 *height)
{
	if (width)
		*width = ddev->physical_width;
	if (height)
		*height = ddev->physical_height;
}

static int mcde_display_set_power_mode_default(struct mcde_display_device *ddev,
	enum mcde_display_power_mode power_mode)
{
	int ret = 0;

	/* OFF -> STANDBY */
	if (ddev->power_mode == MCDE_DISPLAY_PM_OFF &&
		power_mode != MCDE_DISPLAY_PM_OFF) {
		if (ddev->platform_enable) {
			ret = ddev->platform_enable(ddev);
			if (ret)
				return ret;
		}
		ddev->power_mode = MCDE_DISPLAY_PM_STANDBY;
		/* force register settings */
		if (ddev->port->type == MCDE_PORTTYPE_DPI)
			ddev->update_flags = UPDATE_FLAG_VIDEO_MODE |
						UPDATE_FLAG_PIXEL_FORMAT;
	}

	if (ddev->port->type == MCDE_PORTTYPE_DSI) {
		/* STANDBY -> ON */
		if (ddev->power_mode == MCDE_DISPLAY_PM_STANDBY &&
			power_mode == MCDE_DISPLAY_PM_ON) {
			ret = mcde_dsi_dcs_write(ddev->chnl_state,
				DCS_CMD_EXIT_SLEEP_MODE, NULL, 0);
			if (ret)
				return ret;

			ret = mcde_dsi_dcs_write(ddev->chnl_state,
				DCS_CMD_SET_DISPLAY_ON, NULL, 0);
			if (ret)
				return ret;

			ddev->power_mode = MCDE_DISPLAY_PM_ON;
		} else if (ddev->power_mode == MCDE_DISPLAY_PM_ON &&
			power_mode <= MCDE_DISPLAY_PM_STANDBY) {
			/* ON -> STANDBY */
			ret = mcde_dsi_dcs_write(ddev->chnl_state,
				DCS_CMD_SET_DISPLAY_OFF, NULL, 0);
			if (ret)
				return ret;

			ret = mcde_dsi_dcs_write(ddev->chnl_state,
				DCS_CMD_ENTER_SLEEP_MODE, NULL, 0);
			if (ret)
				return ret;

			ddev->power_mode = MCDE_DISPLAY_PM_STANDBY;
		}
	} else if (ddev->port->type == MCDE_PORTTYPE_DPI) {
		ddev->power_mode = power_mode;
	} else if (ddev->power_mode != power_mode) {
		return -EINVAL;
	}

	/* SLEEP -> OFF */
	if (ddev->power_mode == MCDE_DISPLAY_PM_STANDBY &&
		power_mode == MCDE_DISPLAY_PM_OFF) {
		if (ddev->platform_disable) {
			ret = ddev->platform_disable(ddev);
			if (ret)
				return ret;
		}
		ddev->power_mode = MCDE_DISPLAY_PM_OFF;
	}

	mcde_chnl_set_power_mode(ddev->chnl_state, ddev->power_mode);

	return ret;
}

static inline enum mcde_display_power_mode mcde_display_get_power_mode_default(
	struct mcde_display_device *ddev)
{
	return ddev->power_mode;
}

static inline int mcde_display_try_video_mode_default(
	struct mcde_display_device *ddev,
	struct mcde_video_mode *video_mode)
{
	/*
	 * DSI video mode:
	 * This function is intended for configuring supported video mode(s).
	 * Overload it into the panel driver file and set up blanking
	 * intervals and pixel clock according to below recommendations.
	 *
	 * vertical blanking parameters vbp, vfp, vsw are given in lines
	 * horizontal blanking parameters hbp, hfp, hsw are given in pixels
	 *
	 * video_mode->pixclock is the time between two pixels (in picoseconds)
	 * The source of the pixel clock is DSI PLL and it shall be set to
	 * meet the requirement
	 *
	 * non-burst mode:
	 * pixel clock (Hz) = (VACT+VBP+VFP+VSA) * (HACT+HBP+HFP+HSA) *
	 *                    framerate * bpp / num_data_lanes
	 *
	 * burst mode:
	 * pixel clock (Hz) > (VACT+VBP+VFP+VSA) * (HACT+HBP+HFP+HSA) *
	 *                    framerate * bpp / num_data_lanes * 1.1
	 * (1.1 is a 10% margin needed for burst mode calculations)
	 */
	return 0;
}

static int mcde_display_set_video_mode_default(struct mcde_display_device *ddev,
	struct mcde_video_mode *video_mode)
{
	int ret;
	struct mcde_video_mode channel_video_mode;

	if (!video_mode)
		return -EINVAL;

	ddev->video_mode = *video_mode;
	channel_video_mode = ddev->video_mode;
	/*
	 * If orientation is 90 or 270 then rotate the x and y resolution
	 * Channel video mode resolution is always the native display
	 * resolution
	 */
	if (ddev->orientation == MCDE_DISPLAY_ROT_90_CCW ||
				ddev->orientation == MCDE_DISPLAY_ROT_90_CW) {
		u32 old_x_res;

		old_x_res = channel_video_mode.xres;
		channel_video_mode.xres = channel_video_mode.yres;
		channel_video_mode.yres = old_x_res;
	}
	ret = mcde_chnl_set_video_mode(ddev->chnl_state, &channel_video_mode);
	if (ret < 0) {
		dev_warn(&ddev->dev, "%s:Failed to set video mode\n", __func__);
		return ret;
	}

	ddev->update_flags |= UPDATE_FLAG_VIDEO_MODE;

	return 0;
}

static inline void mcde_display_get_video_mode_default(
	struct mcde_display_device *ddev, struct mcde_video_mode *video_mode)
{
	if (video_mode)
		*video_mode = ddev->video_mode;
}

static int mcde_display_set_pixel_format_default(
	struct mcde_display_device *ddev, enum mcde_ovly_pix_fmt format)
{
	int ret;

	ddev->pixel_format = format;
	ret = mcde_chnl_set_pixel_format(ddev->chnl_state,
						ddev->port->pixel_format);
	if (ret < 0) {
		dev_warn(&ddev->dev, "%s:Failed to set pixel format = %d\n",
							__func__, format);
		return ret;
	}

	return 0;
}

static inline enum mcde_ovly_pix_fmt mcde_display_get_pixel_format_default(
	struct mcde_display_device *ddev)
{
	return ddev->pixel_format;
}

static int mcde_display_set_rotation_default(struct mcde_display_device *ddev,
	enum mcde_display_rotation rotation)
{
	int ret;
	u8 param = 0;
	enum mcde_display_rotation final;
	struct mcde_port *port = ddev->port;
	bool use_default;

	final = (360 + rotation - ddev->orientation) % 360;
	ret = mcde_chnl_set_rotation(ddev->chnl_state, final);
	if (WARN_ON(ret))
		return ret;

	if (port->dcs_addr_mode.normal || port->dcs_addr_mode.hor_flip)
		use_default = false; /* Use values set in disp driver */
	else
		use_default = true; /* Not set in disp driver; use default */

	if (final == MCDE_DISPLAY_ROT_180_CW) {
		if (use_default)
			param = DSI_DCS_ADDR_MODE_HOR_FLIP_DEFAULT;
		else
			param = port->dcs_addr_mode.hor_flip;
	} else {
		if (use_default)
			param = DSI_DCS_ADDR_MODE_NORMAL_DEFAULT;
		else
			param = port->dcs_addr_mode.normal;
	}

	/* Set normal or horizontal flip */
	(void)mcde_dsi_dcs_write(ddev->chnl_state, DCS_CMD_SET_ADDRESS_MODE,
								&param, 1);

	ddev->rotation = rotation;
	ddev->update_flags |= UPDATE_FLAG_ROTATION;

	return 0;
}

static inline enum mcde_display_rotation mcde_display_get_rotation_default(
	struct mcde_display_device *ddev)
{
	return ddev->rotation;
}

static int mcde_display_apply_config_default(struct mcde_display_device *ddev)
{
	int ret;

	if (!ddev->update_flags)
		return 0;

	if (ddev->update_flags &
			(UPDATE_FLAG_VIDEO_MODE | UPDATE_FLAG_ROTATION))
		mcde_chnl_stop_flow(ddev->chnl_state);

	ret = mcde_chnl_apply(ddev->chnl_state);
	if (ret < 0) {
		dev_warn(&ddev->dev, "%s:Failed to apply to channel\n",
							__func__);
		return ret;
	}
	ddev->update_flags = 0;
	ddev->first_update = true;

	return 0;
}

static int mcde_display_update_default(struct mcde_display_device *ddev,
							bool tripple_buffer)
{
	int ret = 0;

	ret = mcde_chnl_update(ddev->chnl_state, tripple_buffer);

	if (ret < 0) {
		dev_warn(&ddev->dev, "%s:Failed to update channel\n", __func__);
		return ret;
	}
	if (ddev->first_update && ddev->on_first_update)
		ddev->on_first_update(ddev);

	if (ddev->power_mode != MCDE_DISPLAY_PM_ON && ddev->set_power_mode) {
		ret = ddev->set_power_mode(ddev, MCDE_DISPLAY_PM_ON);
		if (ret < 0) {
			dev_warn(&ddev->dev,
				"%s:Failed to set power mode to on\n",
				__func__);
			return ret;
		}
	}

	dev_vdbg(&ddev->dev, "Overlay updated, chnl=%d\n", ddev->chnl_id);

	return 0;
}

static inline int mcde_display_on_first_update_default(
					struct mcde_display_device *ddev)
{
	ddev->first_update = false;
	return 0;
}

static inline bool mcde_display_secure_output_default(void)
{
	return true;
}

void mcde_display_init_device(struct mcde_display_device *ddev)
{
	/* Setup default callbacks */
	ddev->get_native_resolution =
				mcde_display_get_native_resolution_default;
	ddev->get_default_pixel_format =
				mcde_display_get_default_pixel_format_default;
	ddev->get_physical_size = mcde_display_get_physical_size_default;
	ddev->set_power_mode = mcde_display_set_power_mode_default;
	ddev->get_power_mode = mcde_display_get_power_mode_default;
	ddev->try_video_mode = mcde_display_try_video_mode_default;
	ddev->set_video_mode = mcde_display_set_video_mode_default;
	ddev->get_video_mode = mcde_display_get_video_mode_default;
	ddev->set_pixel_format = mcde_display_set_pixel_format_default;
	ddev->get_pixel_format = mcde_display_get_pixel_format_default;
	ddev->set_rotation = mcde_display_set_rotation_default;
	ddev->get_rotation = mcde_display_get_rotation_default;
	ddev->apply_config = mcde_display_apply_config_default;
	ddev->update = mcde_display_update_default;
	ddev->on_first_update = mcde_display_on_first_update_default;
	ddev->secure_output = mcde_display_secure_output_default;

	mutex_init(&ddev->display_lock);
}
