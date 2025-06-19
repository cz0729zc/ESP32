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

// 将RMT读取到的脉冲数据处理为温度和湿度(rmt_symbol_word_t称为RMT符号)
static int parse_items(rmt_symbol_word_t *item, int item_num, int *humidity, int *temp_x10)
{
	int i = 0;
	unsigned int rh = 0, temp = 0, checksum = 0;
	if (item_num < 41){					// 检查是否有足够的脉冲数
		//ESP_LOGI(TAG, "item_num < 41  %d",item_num);
		return 0;
	}
	if(item_num > 41)
		item++;								// 跳过开始信号脉冲

	for (i = 0; i < 16; i++, item++)	// 提取湿度数据
	{
		uint16_t duration = 0;
		if(item->level0)
			duration = item->duration0;
		else
			duration = item->duration1;
		rh = (rh << 1) + (duration < 35 ? 0 : 1);
	}


	for (i = 0; i < 16; i++, item++)	// 提取温度数据
	{
		uint16_t duration = 0;
		if(item->level0)
			duration = item->duration0;
		else
			duration = item->duration1;
		temp = (temp << 1) + (duration < 35 ? 0 : 1);
	}
	
	for (i = 0; i < 8; i++, item++){	// 提取校验数据
	
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

	//判断数据合法性
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