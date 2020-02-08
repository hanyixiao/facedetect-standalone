/* Copyright 2018 Canaan Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include <stdio.h>
#include <string.h>
#include "board_config.h"
#include "dvp.h"
#include "fpioa.h"
#include "lcd.h"
#include "nt35310.h"
#include "ov2640.h"
#include "ov5640.h"
#include "plic.h"
#include "sysctl.h"
#include "uarths.h"
#include "unistd.h"
// ai
#include <image_process.h>
#include <region_layer.h>
// flash
#include <flash-manager.h>
#include <w25qxx.h>

uint32_t g_lcd_gram0[38400] __attribute__((aligned(64)));
uint32_t g_lcd_gram1[38400] __attribute__((aligned(64)));
//
static image_t kpu_image, display_image;

kpu_model_context_t face_detect_task;
static region_layer_t face_detect_rl;
static obj_info_t face_detect_info;
#define ANCHOR_NUM 5
static float anchor[ANCHOR_NUM * 2] = {1.889,    2.5245, 2.9465,   3.94056,
                                       3.99987,  5.3658, 5.155437, 6.92275,
                                       6.718375, 9.01025};

uint8_t model_data[KMODEL_SIZE];

static obj_info_t face_detect_info;

volatile uint32_t g_ai_done_flag;

volatile uint8_t g_dvp_finish_flag;
volatile uint8_t g_ram_mux;

static void draw_edge(uint32_t *gram, obj_info_t *obj_info, uint32_t index,
                      uint16_t color) {
  uint32_t data = ((uint32_t)color << 16) | (uint32_t)color;
  uint32_t *addr1, *addr2, *addr3, *addr4, x1, y1, x2, y2;

  x1 = obj_info->obj[index].x1;
  y1 = obj_info->obj[index].y1;
  x2 = obj_info->obj[index].x2;
  y2 = obj_info->obj[index].y2;

  if (x1 <= 0) x1 = 1;
  if (x2 >= 319) x2 = 318;
  if (y1 <= 0) y1 = 1;
  if (y2 >= 239) y2 = 238;

  addr1 = gram + (320 * y1 + x1) / 2;
  addr2 = gram + (320 * y1 + x2 - 8) / 2;
  addr3 = gram + (320 * (y2 - 1) + x1) / 2;
  addr4 = gram + (320 * (y2 - 1) + x2 - 8) / 2;
  for (uint32_t i = 0; i < 4; i++) {
    *addr1 = data;
    *(addr1 + 160) = data;
    *addr2 = data;
    *(addr2 + 160) = data;
    *addr3 = data;
    *(addr3 + 160) = data;
    *addr4 = data;
    *(addr4 + 160) = data;
    addr1++;
    addr2++;
    addr3++;
    addr4++;
  }

  addr1 = gram + (320 * y1 + x1) / 2;
  addr2 = gram + (320 * y1 + x2 - 2) / 2;
  addr3 = gram + (320 * (y2 - 8) + x1) / 2;
  addr4 = gram + (320 * (y2 - 8) + x2 - 2) / 2;
  for (uint32_t i = 0; i < 8; i++) {
    *addr1 = data;
    *addr2 = data;
    *addr3 = data;
    *addr4 = data;
    addr1 += 160;
    addr2 += 160;
    addr3 += 160;
    addr4 += 160;
  }
}

static void ai_done(void *ctx) { g_ai_done_flag = 1; }

static int dvp_irq(void *ctx) {
  if (dvp_get_interrupt(DVP_STS_FRAME_FINISH)) {
    dvp_config_interrupt(DVP_CFG_START_INT_ENABLE | DVP_CFG_FINISH_INT_ENABLE,
                         0);
    dvp_clear_interrupt(DVP_STS_FRAME_FINISH);
    g_dvp_finish_flag = 1;
  } else {
    dvp_start_convert();
    dvp_clear_interrupt(DVP_STS_FRAME_START);
  }
  return 0;
}

static void io_mux_init(void) {
  /* Init DVP IO map and function settings */
  fpioa_set_function(42, FUNC_CMOS_RST);
  fpioa_set_function(44, FUNC_CMOS_PWDN);
  fpioa_set_function(46, FUNC_CMOS_XCLK);
  fpioa_set_function(43, FUNC_CMOS_VSYNC);
  fpioa_set_function(45, FUNC_CMOS_HREF);
  fpioa_set_function(47, FUNC_CMOS_PCLK);
  fpioa_set_function(41, FUNC_SCCB_SCLK);
  fpioa_set_function(40, FUNC_SCCB_SDA);

  /* Init SPI IO map and function settings */
  fpioa_set_function(38, FUNC_GPIOHS0 + DCX_GPIONUM);
  fpioa_set_function(36, FUNC_SPI0_SS3);
  fpioa_set_function(39, FUNC_SPI0_SCLK);
  fpioa_set_function(37, FUNC_GPIOHS0 + RST_GPIONUM);

  sysctl_set_spi0_dvp_data(1);
}

static void io_set_power(void) {
  /* Set dvp and spi pin to 1.8V */
  sysctl_set_power_mode(SYSCTL_POWER_BANK6, SYSCTL_POWER_V18);
  sysctl_set_power_mode(SYSCTL_POWER_BANK7, SYSCTL_POWER_V18);
}

int main(void) {
  /* Set CPU and dvp clk */
  sysctl_pll_set_freq(SYSCTL_PLL0, 800000000UL);
  sysctl_pll_set_freq(SYSCTL_PLL1, 160000000UL);
  sysctl_pll_set_freq(SYSCTL_PLL2, 45158400UL);
  uarths_init();

  io_mux_init();
  io_set_power();
  plic_init();

  printf("flash init\n");
  w25qxx_init(3, 0, 60000000);

  w25qxx_read_data(KMODEL_START, model_data, KMODEL_SIZE);

  /* LCD init */
  printf("LCD init\n");
  lcd_init();
  lcd_draw_string(136, 70, "DEMO", WHITE);
  lcd_draw_string(104, 150, "face detection", WHITE);

  /* DVP init */
  printf("DVP init\n");
#if OV5640
  dvp_init(16);
  dvp_set_xclk_rate(50000000);
  dvp_enable_burst();
  dvp_set_output_enable(0, 1);
  dvp_set_output_enable(1, 1);
  dvp_set_image_format(DVP_CFG_RGB_FORMAT);
  dvp_set_image_size(320, 240);
  ov5640_init();

#if 0
//    OV5640_Focus_Init();
    OV5640_Light_Mode(2);      //set auto
    OV5640_Color_Saturation(6); //default
    OV5640_Brightness(8);   //default
    OV5640_Contrast(3);     //default
//    OV5640_Sharpness(33);   //set auto
//    OV5640_Auto_Focus();
#endif

#else
  dvp_init(8);
  dvp_set_xclk_rate(24000000);
  dvp_enable_burst();
  dvp_set_output_enable(0, 1);
  dvp_set_output_enable(1, 1);
  dvp_set_image_format(DVP_CFG_RGB_FORMAT);

  dvp_set_image_size(320, 240);
  ov2640_init();
#endif

  kpu_image.pixel = 3;
  kpu_image.width = 320;
  kpu_image.height = 240;
  image_init(&kpu_image);
  display_image.pixel = 2;
  display_image.width = 320;
  display_image.height = 240;
  image_init(&display_image);

  dvp_set_ai_addr((uint32_t)kpu_image.addr,
                  (uint32_t)(kpu_image.addr + 320 * 240),
                  (uint32_t)(kpu_image.addr + 320 * 240 * 2));
  dvp_set_display_addr((uint32_t)display_image.addr);
  dvp_config_interrupt(DVP_CFG_START_INT_ENABLE | DVP_CFG_FINISH_INT_ENABLE, 0);
  dvp_disable_auto();

  /* DVP interrupt config */
  printf("DVP interrupt config\n");
  plic_set_priority(IRQN_DVP_INTERRUPT, 1);
  plic_irq_register(IRQN_DVP_INTERRUPT, dvp_irq, NULL);
  plic_irq_enable(IRQN_DVP_INTERRUPT);

  /* init face detect model */
  if (kpu_load_kmodel(&face_detect_task, model_data) != 0) {
    printf("\nmodel init error\n");
    while (1)
      ;
  }
  face_detect_rl.anchor_number = ANCHOR_NUM;
  face_detect_rl.anchor = anchor;
  face_detect_rl.threshold = 0.7;
  face_detect_rl.nms_value = 0.3;
  region_layer_init(&face_detect_rl, 20, 15, 30, kpu_image.width,
                    kpu_image.height);
  /* enable global interrupt */
  sysctl_enable_irq();

  /* system start */
  printf("system start\n");

  while (1) {
    g_dvp_finish_flag = 0;
    dvp_clear_interrupt(DVP_STS_FRAME_START | DVP_STS_FRAME_FINISH);
    dvp_config_interrupt(DVP_CFG_START_INT_ENABLE | DVP_CFG_FINISH_INT_ENABLE,
                         1);
    while (g_dvp_finish_flag == 0)
      ;
    /* run face detect */
    g_ai_done_flag = 0;
    kpu_run_kmodel(&face_detect_task, kpu_image.addr, DMAC_CHANNEL5, ai_done,
                   NULL);
    while (!g_ai_done_flag)
      ;
    float *output;
    size_t output_size;
    kpu_get_output(&face_detect_task, 0, (uint8_t **)&output, &output_size);
    face_detect_rl.input = output;
    region_layer_run(&face_detect_rl, &face_detect_info);
    /* run key point detect */
    for (uint32_t face_cnt = 0; face_cnt < face_detect_info.obj_number;
         face_cnt++) {
      draw_edge((uint32_t *)display_image.addr, &face_detect_info, face_cnt,
                BLUE);
    }
    /* display result */
    lcd_draw_picture(0, 0, 320, 240, (uint32_t *)display_image.addr);
  }

  return 0;
}
