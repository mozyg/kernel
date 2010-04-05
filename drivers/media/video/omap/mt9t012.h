/*
 * drivers/media/video/omap/mt9t012.h
 *
 * Copyright (C) 2007 Texas Instruments.
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 * Copyright (C) 2007 Texas Instruments.
 *
 * Register defines for Micron MT9T012 raw image sensor  
 *
 */

#ifndef MT9T012_H
#define MT9T012_H

/* The ID values we are looking for */
#define MT9T012_MOD_ID			0x1600
#define MT9T012_REV			0x00
#define MT9T012_MFR_ID			0x06


/* Used registers */
#define REG_MODEL_ID			0x0000 
#define REG_REVISION_NUMBER		0x0002
#define REG_MANUFACTURER_ID		0x0003

#define REG_MODE_SELECT		0x0100 
#define REG_IMAGE_ORIENTATION		0x0101 
#define REG_SOFTWARE_RESET		0x0103
#define REG_GROUPED_PAR_HOLD		0x0104

#define REG_FINE_INT_TIME		0x0200
#define REG_COARSE_INT_TIME		0x0202

#define REG_ANALOG_GAIN_GLOBAL		0x0204
#define REG_ANALOG_GAIN_GREENR		0x0206
#define REG_ANALOG_GAIN_RED		0x0208
#define REG_ANALOG_GAIN_BLUE		0x020A
#define REG_ANALOG_GAIN_GREENB		0x020C
#define REG_DIGITAL_GAIN_GREENR	0x020E
#define REG_DIGITAL_GAIN_RED		0x0210
#define REG_DIGITAL_GAIN_BLUE		0x0212
#define REG_DIGITAL_GAIN_GREENB	0x0214

#define REG_VT_PIX_CLK_DIV		0x0300
#define REG_VT_SYS_CLK_DIV		0x0302
#define REG_PRE_PLL_CLK_DIV		0x0304
#define REG_PLL_MULTIPLIER		0x0306
#define REG_OP_PIX_CLK_DIV		0x0308
#define REG_OP_SYS_CLK_DIV		0x030A

#define REG_FRAME_LEN_LINES		0x0340
#define REG_LINE_LEN_PCK		0x0342

#define REG_X_ADDR_START		0x0344
#define REG_Y_ADDR_START		0x0346
#define REG_X_ADDR_END			0x0348
#define REG_Y_ADDR_END			0x034A
#define REG_X_OUTPUT_SIZE		0x034C
#define REG_Y_OUTPUT_SIZE		0x034E

#define REG_SCALING_MODE		0x0400
#define REG_SCALE_M			0x0404
#define REG_SCALE_N			0x0406

#define REG_ROW_SPEED			0x3016
#define REG_RESET_REGISTER		0x301A
#define REG_PIXEL_ORDER		0x3024
#define REG_READ_MODE			0x3040

#define REG_DATAPATH_STATUS		0x306A
#define REG_DATAPATH_SELECT		0x306E


#endif /* ifndef MT9T012_H */

