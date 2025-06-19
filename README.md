# ESP32 学习笔记

该项目包含了我学习时候的各类测试工程，并且我会在各类驱动文件的开始部分加上我自己的理解和我的疑惑和解答

## DHT11

~~~c
/* 我的理解：zc
 * DHT11.c
 * 用户调用DHT11_Init()函数,打开rmt接收通道,注册回调函数,使能通道
 * 然后用户调用DHT11_StartGet()函数，发送DHT11开始信号，并启动RMT接收器以获取数据
 * 然后rmt通道开启后就会通过rmt_receive函数接收来自DHT11的数据并保存再缓冲区中,
 * 当数据接收完成后,就调用example_rmt_rx_done_callback()回调函数处理接收到的数据(过程有rmt模块自动完成不需要cpu响应)
 * 在回调函数中将user_data数据发送数据接收队列rx_receive_queue中,然后再主函数中使用xQueueReceive监听队列获取数据并将数据保存再rx_data中进行处理
 * 然后再主函数中调用parse_items()函数处理数据
 * 
 * 问题及回答：
 * 问：为什么不在example_rmt_rx_done_callback函数中调用parse_items()进行处理，而是大费周章使用队列传输数据,非要在主函数中调用parse_items()?)
 * 答：因为example_rmt_rx_done_callback是作为中断服务函数，而parse_items()函数处理需要耗时或需要调度（中断上下文不能做复杂操作（如内存分配、延时、调度等））
 * 问：为什么要使用队列？
 * 答：队列用于 在 RMT 接收完成中断回调函数与主任务之间安全地传递数据，实现异步非阻塞的数据解析， 中断上下文和任务上下文之间安全高效地传递 RMT 接收结果，
 * 避免在中断中做复杂操作，提升系统的稳定性与响应能力。
 * 
 * 
 */


#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/rmt_rx.h>
#include <driver/rmt_tx.h>
#include <soc/rmt_reg.h>
#include "esp_log.h"
#include "driver/gpio.h"
#include "esp_system.h"
#include "esp32/rom/ets_sys.h"

#define TAG		"DHT11"

uint8_t DHT11_PIN = -1;

//rmt接收通道句柄
static rmt_channel_handle_t rx_chan_handle = NULL;

//数据接收队列
static QueueHandle_t rx_receive_queue = NULL;

// 将RMT读取到的脉冲数据处理为温度和湿度
static int parse_items(rmt_symbol_word_t *item, int item_num, int *humidity, int *temp_x10);

//接收完成回调函数
static bool IRAM_ATTR example_rmt_rx_done_callback(rmt_channel_handle_t channel, const rmt_rx_done_event_data_t *edata, void *user_data)
{
    BaseType_t high_task_wakeup = pdFALSE;
    QueueHandle_t rx_receive_queue = (QueueHandle_t)user_data;
    // send the received RMT symbols to the parser task
    xQueueSendFromISR(rx_receive_queue, edata, &high_task_wakeup);
    return high_task_wakeup == pdTRUE;
}

/** DHT11初始化
 * @param dht11_pin GPIO引脚
 * @return 无
*/
void DHT11_Init(uint8_t dht11_pin)
{
	DHT11_PIN = dht11_pin;
    
    rmt_rx_channel_config_t rx_chan_config = {
        .clk_src = RMT_CLK_SRC_APB,   // 选择时钟源
        .resolution_hz = 1000 * 1000, 	  // 1 MHz 滴答分辨率，即 1 滴答 = 1 µs
        .mem_block_symbols = 64,          // 内存块大小，即 64 * 4 = 256 字节
        .gpio_num = dht11_pin,            // GPIO 编号
        .flags.invert_in = false,         // 不反转输入信号
        .flags.with_dma = false,          // 不需要 DMA 后端(ESP32S3才有)
    };
	//创建rmt接收通道
	ESP_ERROR_CHECK(rmt_new_rx_channel(&rx_chan_config, &rx_chan_handle));
 
	//新建接收数据队列
    rx_receive_queue = xQueueCreate(20, sizeof(rmt_rx_done_event_data_t));
    assert(rx_receive_queue);

	//注册接收完成回调函数
	 ESP_LOGI(TAG, "register RX done callback");
    rmt_rx_event_callbacks_t cbs = {
        .on_recv_done = example_rmt_rx_done_callback,
    };
    ESP_ERROR_CHECK(rmt_rx_register_event_callbacks(rx_chan_handle, &cbs, rx_receive_queue));

	//使能RMT接收通道
	ESP_ERROR_CHECK(rmt_enable(rx_chan_handle));
}
/**
 * 解析脉冲序列以获取温湿度信息
 * 
 * 本函数负责解析给定的脉冲序列（item数组），从中提取出湿度和温度信息，并通过校验和来验证数据的完整性
 * 主要应用于解码DHT11/DHT22等温湿度传感器的信号
 * 
 * @param item 脉冲序列数组，包含温湿度信息
 * @param item_num 脉冲序列的长度
 * @param humidity 指向一个整型变量的指针，用于存储解析出的湿度值
 * @param temp_x10 指向一个整型变量的指针，用于存储解析出的温度值，乘以10
 * @return 成功解析则返回1，否则返回0
 */
static int parse_items(rmt_symbol_word_t *item, int item_num, int *humidity, int *temp_x10)
{
    int i = 0;
    unsigned int rh = 0, temp = 0, checksum = 0;
    
    // 检查是否有足够的脉冲数
    if (item_num < 41){
        //ESP_LOGI(TAG, "item_num < 41  %d",item_num);
        return 0;
    }
    
    // 跳过可能的额外开始信号脉冲
    if(item_num > 41)
        item++;
    
    // 提取湿度数据
    for (i = 0; i < 16; i++, item++){
        uint16_t duration = 0;
        if(item->level0)
            duration = item->duration0;
        else
            duration = item->duration1;
        rh = (rh << 1) + (duration < 35 ? 0 : 1);
    }
    
    // 提取温度数据
    for (i = 0; i < 16; i++, item++){
        uint16_t duration = 0;
        if(item->level0)
            duration = item->duration0;
        else
            duration = item->duration1;
        temp = (temp << 1) + (duration < 35 ? 0 : 1);
    }
    
    // 提取校验数据
    for (i = 0; i < 8; i++, item++){
        uint16_t duration = 0;
        if(item->level0)
            duration = item->duration0;
        else
            duration = item->duration1;
        checksum = (checksum << 1) + (duration < 35 ? 0 : 1);
    }
    
    // 检查校验
    if ((((temp >> 8) + temp + (rh >> 8) + rh) & 0xFF) != checksum){
        ESP_LOGI(TAG, "Checksum failure %4X %4X %2X\n", temp, rh, checksum);
        return 0;
    }
    
    // 返回数据
    rh = rh >> 8;
    temp = (temp >> 8) * 10 + (temp & 0xFF);
    
    // 判断数据合法性
    if(rh <= 100)
        *humidity = rh;
    if(temp <= 600)
        *temp_x10 = temp;
    
    return 1;
}


/** 获取DHT11数据
 * @param temp_x10 温度值
 * @return 无
*/
int DHT11_StartGet(int *temp_x10, int *humidity)
{
	//发送20ms开始信号脉冲启动DHT11单总线
	gpio_set_direction(DHT11_PIN, GPIO_MODE_OUTPUT);
	gpio_set_level(DHT11_PIN, 1);
	ets_delay_us(1000);
	gpio_set_level(DHT11_PIN, 0);
	ets_delay_us(20000);
	//拉高20us
	gpio_set_level(DHT11_PIN, 1);
	ets_delay_us(20);
	//信号线设置为输入准备接收数据
	gpio_set_direction(DHT11_PIN, GPIO_MODE_INPUT);
	gpio_set_pull_mode(DHT11_PIN,GPIO_PULLUP_ONLY);

	//启动RMT接收器以获取数据
	rmt_receive_config_t receive_config = {
        .signal_range_min_ns = 100,     //最小脉冲宽度(0.1us),信号长度小于这个值，视为干扰
        .signal_range_max_ns = 1000*1000, 	//最大脉冲宽度(1000us)，信号长度大于这个值，视为结束信号
    };
	
	static rmt_symbol_word_t raw_symbols[128];	//接收缓存
    static rmt_rx_done_event_data_t rx_data;	//实际接收到的数据
	ESP_ERROR_CHECK(rmt_receive(rx_chan_handle, raw_symbols, sizeof(raw_symbols), &receive_config));

	// wait for RX done signal
	if (xQueueReceive(rx_receive_queue, &rx_data, pdMS_TO_TICKS(1000)) == pdTRUE) {
		// parse the receive symbols and print the result
		return parse_items(rx_data.received_symbols, rx_data.num_symbols,humidity, temp_x10);
	}
    return 0;
}
~~~

## WS2812

~~~c
/* 我的理解：zc
 * 首先在 main 函数中调用 ws2812_init 函数，为 WS2812 操作句柄 handle 配置基本信息，
 * 包括分配 ws2812_strip_t 结构体内存并初始化其成员（如 LED 数量、GPIO 引脚、RGB 缓冲区等）。
 *
 * 然后调用 rmt_new_led_strip_encoder 函数创建一个自定义的 RMT 编码器，并将其赋值给 handle->led_encoder，
 * 用于控制 WS2812 所需的精确时序。
 *
 * 在 rmt_new_led_strip_encoder 函数中，我们创建了自定义编码器结构体，并设置其回调函数：
 * - .encode = rmt_encode_led_strip
 * - .del = rmt_del_led_strip_encoder
 * - .reset = rmt_led_strip_encoder_reset
 *
 * 当用户调用 rmt_transmit() 发送数据时，RMT 子系统会自动调用 rmt_encode_led_strip() 函数，
 * 将 RGB 用户数据编码为 rmt_symbol_word_t 类型的 RMT 符号，从而驱动 WS2812 LED 显示指定颜色。
 */

/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "esp_check.h"
#include "led_ws2812.h"
#include "driver/rmt_tx.h"

static const char *TAG = "led_encoder";

#define LED_STRIP_RESOLUTION_HZ 10000000 // 10MHz 分辨率, 也就是1tick = 0.1us，也就是可以控制的最小时间单元，低于0.1us的脉冲无法产生

//WS2812驱动的描述符
struct ws2812_strip_t
{
    rmt_channel_handle_t led_chan;          //rmt通道
    rmt_encoder_handle_t led_encoder;       //rmt编码器
    uint8_t *led_buffer;                    //rgb数据
    int led_num;                            //led个数
};

//自定义编码器
typedef struct {
    rmt_encoder_t base;                     //编码器，里面包含三个需要用户实现的回调函数，encode,del,ret
    rmt_encoder_t *bytes_encoder;           //字节编码器，调用rmt_new_bytes_encoder函数后创建
    rmt_encoder_t *copy_encoder;            //拷贝编码器，调用rmt_new_copy_encoder函数后创建
    int state;                              //状态控制
    rmt_symbol_word_t reset_code;           //结束位的时序
} rmt_led_strip_encoder_t;

/* 发送WS2812数据的函数调用顺序如下
 * 1、调用rmt_transmit，需传入RMT通道、发送的数据、编码器参数
 * 2、调用编码器的encode函数，在本例程中就是调用rmt_encode_led_strip函数
 * 3、调用由rmt_new_bytes_encoder创建的字节编码器编码函数bytes_encoder->encode，将用户数据编码成rmt_symbol_word_t RMT符号
 * 4、调用由rmt_new_copy_encoder创建的拷贝编码器编码函数copy_encoder->encode，将复位信号安装既定的电平时间进行编码
 * 5、rmt_encode_led_strip函数返回，在底层将信号发送出去（本质上是操作IO管脚高低电平）
*/

/** 编码回调函数
 * @param encoder 编码器
 * @param channel RMT通道
 * @param primary_data 待编码用户数据
 * @param data_size 待编码用户数据长度
 * @param ret_state 编码状态
 * @return RMT符号个数
*/
static size_t rmt_encode_led_strip(rmt_encoder_t *encoder, rmt_channel_handle_t channel, const void *primary_data, size_t data_size, rmt_encode_state_t *ret_state)
{
    /*
    __containerof宏的作用:
    通过结构的成员来访问这个结构的地址
    在这个函数中，传入参数encoder是rmt_led_strip_encoder_t结构体中的base成员
    __containerof宏通过encoder的地址，根据rmt_led_strip_encoder_t的内存排布找到rmt_led_strip_encoder_t* 的首地址
    */
    rmt_led_strip_encoder_t *led_encoder = __containerof(encoder, rmt_led_strip_encoder_t, base);
    rmt_encoder_handle_t bytes_encoder = led_encoder->bytes_encoder;        //取出字节编码器
    rmt_encoder_handle_t copy_encoder = led_encoder->copy_encoder;          //取出拷贝编码器
    rmt_encode_state_t session_state = RMT_ENCODING_RESET;
    rmt_encode_state_t state = RMT_ENCODING_RESET;
    size_t encoded_symbols = 0;
    switch (led_encoder->state) {   //led_encoder->state是自定义的状态，这里只有两种值，0是发送RGB数据，1是发送复位码
    case 0: // send RGB data
        encoded_symbols += bytes_encoder->encode(bytes_encoder, channel, primary_data, data_size, &session_state);
        if (session_state & RMT_ENCODING_COMPLETE) {    //字节编码完成
            led_encoder->state = 1; // switch to next state when current encoding session finished
        }
        if (session_state & RMT_ENCODING_MEM_FULL) {    //缓存不足，本次退出
            state |= RMT_ENCODING_MEM_FULL;
            goto out; // yield if there's no free space for encoding artifacts
        }
    // fall-through
    case 1: // send reset code
        encoded_symbols += copy_encoder->encode(copy_encoder, channel, &led_encoder->reset_code,
                                                sizeof(led_encoder->reset_code), &session_state);
        if (session_state & RMT_ENCODING_COMPLETE) {
            led_encoder->state = RMT_ENCODING_RESET; // back to the initial encoding session
            state |= RMT_ENCODING_COMPLETE;
        }
        if (session_state & RMT_ENCODING_MEM_FULL) {
            state |= RMT_ENCODING_MEM_FULL;
            goto out; // yield if there's no free space for encoding artifacts
        }
    }
out:
    *ret_state = state;
    return encoded_symbols;
}

static esp_err_t rmt_del_led_strip_encoder(rmt_encoder_t *encoder)
{
    rmt_led_strip_encoder_t *led_encoder = __containerof(encoder, rmt_led_strip_encoder_t, base);
    rmt_del_encoder(led_encoder->bytes_encoder);
    rmt_del_encoder(led_encoder->copy_encoder);
    free(led_encoder);
    return ESP_OK;
}

static esp_err_t rmt_led_strip_encoder_reset(rmt_encoder_t *encoder)
{
    rmt_led_strip_encoder_t *led_encoder = __containerof(encoder, rmt_led_strip_encoder_t, base);
    rmt_encoder_reset(led_encoder->bytes_encoder);
    rmt_encoder_reset(led_encoder->copy_encoder);
    led_encoder->state = RMT_ENCODING_RESET;
    return ESP_OK;
}

/** 创建一个基于WS2812时序的编码器
 * @param ret_encoder 返回的编码器，这个编码器在使用rmt_transmit函数传输时会用到
 * @return ESP_OK or ESP_FAIL
*/
esp_err_t rmt_new_led_strip_encoder(rmt_encoder_handle_t *ret_encoder)
{
    esp_err_t ret = ESP_OK;

    //创建一个自定义的编码器结构体，用于控制发送编码的流程
    rmt_led_strip_encoder_t *led_encoder = NULL;
    led_encoder = calloc(1, sizeof(rmt_led_strip_encoder_t));
    ESP_GOTO_ON_FALSE(led_encoder, ESP_ERR_NO_MEM, err, TAG, "no mem for led strip encoder");
    led_encoder->base.encode = rmt_encode_led_strip;    //这个函数会在rmt发送数据的时候被调用，我们可以在这个函数增加额外代码进行控制
    led_encoder->base.del = rmt_del_led_strip_encoder;  //这个函数在卸载rmt时被调用
    led_encoder->base.reset = rmt_led_strip_encoder_reset;  //这个函数在复位rmt时被调用

    //新建一个编码器配置(0,1位持续时间，参考芯片手册)
    rmt_bytes_encoder_config_t bytes_encoder_config = {
        .bit0 = {
            .level0 = 1,
            .duration0 = 0.3 * LED_STRIP_RESOLUTION_HZ / 1000000, // T0H=0.3us
            .level1 = 0,
            .duration1 = 0.9 * LED_STRIP_RESOLUTION_HZ / 1000000, // T0L=0.9us
        },
        .bit1 = {
            .level0 = 1,
            .duration0 = 0.9 * LED_STRIP_RESOLUTION_HZ / 1000000, // T1H=0.9us
            .level1 = 0,
            .duration1 = 0.3 * LED_STRIP_RESOLUTION_HZ / 1000000, // T1L=0.3us
        },
        .flags.msb_first = 1 //高位先传输
    };
    //传入编码器配置，获得数据编码器操作句柄
    rmt_new_bytes_encoder(&bytes_encoder_config, &led_encoder->bytes_encoder);

    //新建一个拷贝编码器配置，拷贝编码器一般用于传输恒定的字符数据，比如说结束位
    rmt_copy_encoder_config_t copy_encoder_config = {};
    rmt_new_copy_encoder(&copy_encoder_config, &led_encoder->copy_encoder);

    //设定结束位时序
    uint32_t reset_ticks = LED_STRIP_RESOLUTION_HZ / 1000000 * 50 / 2; //分辨率/1M=每个ticks所需的us，然后乘以50就得出50us所需的ticks
    led_encoder->reset_code = (rmt_symbol_word_t) {
        .level0 = 0,
        .duration0 = reset_ticks,
        .level1 = 0,
        .duration1 = reset_ticks,
    };

    //返回编码器
    *ret_encoder = &led_encoder->base;
    return ESP_OK;
err:
    if (led_encoder) {
        if (led_encoder->bytes_encoder) {
            rmt_del_encoder(led_encoder->bytes_encoder);
        }
        if (led_encoder->copy_encoder) {
            rmt_del_encoder(led_encoder->copy_encoder);
        }
        free(led_encoder);
    }
    return ret;
}

/** 初始化WS2812外设
 * @param gpio 控制WS2812的管脚
 * @param maxled 控制WS2812的个数
 * @param led_handle 返回的控制句柄
 * @return ESP_OK or ESP_FAIL
*/
esp_err_t ws2812_init(gpio_num_t gpio,int maxled,ws2812_strip_handle_t* handle)
{
    struct ws2812_strip_t* led_handle = NULL;
    //新增一个WS2812驱动描述
    led_handle = calloc(1, sizeof(struct ws2812_strip_t));
    assert(led_handle);
    //按照led个数来分配RGB缓存数据
    led_handle->led_buffer = calloc(1,maxled*3);
    assert(led_handle->led_buffer);
    //设置LED个数
    led_handle->led_num = maxled;

    //定义一个RMT发送通道配置
    rmt_tx_channel_config_t tx_chan_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,         //默认时钟源
        .gpio_num = gpio,                       //GPIO管脚
        .mem_block_symbols = 64,                //内存块大小，即 64 * 4 = 256 字节
        .resolution_hz = LED_STRIP_RESOLUTION_HZ,   //RMT通道的分辨率10000000hz=0.1us，也就是可以控制的最小时间单元
        .trans_queue_depth = 4,                 //底层后台发送的队列深度
    };

    //创建一个RMT发送通道
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_config, &led_handle->led_chan));

    //创建自定义编码器（重点函数），所谓编码，就是发射红外时加入我们的时序控制
    ESP_ERROR_CHECK(rmt_new_led_strip_encoder(&led_handle->led_encoder));

    //使能RMT通道
    ESP_ERROR_CHECK(rmt_enable(led_handle->led_chan));

    //返回WS2812操作句柄
    *handle = led_handle;

    return ESP_OK;
}

/** 反初始化WS2812外设
 * @param handle 初始化的句柄
 * @return ESP_OK or ESP_FAIL
*/
esp_err_t ws2812_deinit(ws2812_strip_handle_t handle)
{
    if(!handle)
        return ESP_OK;
    rmt_del_encoder(handle->led_encoder);
    if(handle->led_buffer)
        free(handle->led_buffer);
    free(handle);
    return ESP_OK;
}

/** 向某个WS2812写入RGB数据
 * @param handle 句柄
 * @param index 第几个WS2812（0开始）
 * @param r,g,b RGB数据
 * @return ESP_OK or ESP_FAIL
*/
esp_err_t ws2812_write(ws2812_strip_handle_t handle,uint32_t index,uint32_t r,uint32_t g,uint32_t b)
{
     rmt_transmit_config_t tx_config = {
        .loop_count = 0, //不循环发送
    };
    if(index >= handle->led_num)
        return ESP_FAIL;
    uint32_t start = index*3;
    handle->led_buffer[start+0] = g & 0xff;     //注意，WS2812的数据顺序时GRB
    handle->led_buffer[start+1] = r & 0xff;
    handle->led_buffer[start+2] = b & 0xff;

    return rmt_transmit(handle->led_chan, handle->led_encoder, handle->led_buffer, handle->led_num*3, &tx_config);
    
}


~~~

