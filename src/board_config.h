#ifndef _BOARD_CONFIG_
#define _BOARD_CONFIG_

#define OV5640 0
#define OV2640 1
#define BOARD_AIRV 1

#if (OV5640 && OV2640) || (!OV5640 && !OV2640)
#error ov sensor only choose one
#endif

#endif
