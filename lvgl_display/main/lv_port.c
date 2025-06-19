#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "esp_timer.h"

#include "lvgl.h"
#include "cst816t_driver.h"
#include "st7789_driver.h"


/*
1.初始化和注册LVGL显示驱动
2.初始化和注册LVGL输入设备驱动
3.初始化ST7789显示驱动
4.初始化CST816T触摸屏驱动
5.提供1ms定时器给LVGL使用
*/

#define TAG "lv_port"

// LCD显示状态：1表示竖屏，0表示横屏
#define LCDSHOWSTATE 1

#if LCDSHOWSTATE
// 竖屏模式
#define LCD_WIDTH  240
#define LCD_HEIGHT 280
#else
// 横屏模式
#define LCD_WIDTH  280
#define LCD_HEIGHT 240
#endif


static lv_disp_drv_t disp_drv;

//显示驱动函数
void disp_flush(struct _lv_disp_drv_t * disp_drv, const lv_area_t * area, lv_color_t * color_p)
{
#if LCDSHOWSTATE
    // 竖屏模式下正常刷新
    st7789_flush(area->x1, area->x2 + 1, area->y1 + 20, area->y2 + 1 + 20, color_p);
#else
    // 横屏模式下交换x和y，并调整坐标映射
    st7789_flush(area->x1+20, area->x2+20 + 1, area->y1, area->y2 + 1, color_p);
#endif
}

//输入驱动函数
void IRAM_ATTR indev_read(struct _lv_indev_drv_t * indev_drv, lv_indev_data_t * data)
{
    int state;
    lv_coord_t x, y;

    // 获取原始触摸坐标
    cst816t_read(&x, &y, &state);

#if !LCDSHOWSTATE
    // 如果是横屏，交换x/y，并做适当翻转以匹配屏幕方向
    lv_coord_t rotated_x = y;
    lv_coord_t rotated_y = LCD_HEIGHT - x - 1;

    data->point.x = rotated_x;
    data->point.y = rotated_y;
#else
    // 竖屏直接使用原始值
    data->point.x = LCD_WIDTH - x - 1;
    data->point.y =LCD_HEIGHT - y - 1; 
#endif

    data->state = state;
}
/**
 * lv_flush_done_cb函数是LVGL库中的一个回调函数，用于通知LVGL框架显示刷新已完成。
 * 这个函数在图形界面渲染流程中起到关键作用，特别是在使用双缓冲技术的显示设备上。
 * 当显示内容刷新到屏幕上后，该函数会被调用，以通知LVGL可以开始处理下一帧的渲染。
 * 
 * @param param 一个指向lv_disp_drv_t结构体的指针，该结构体包含了显示设备的驱动信息。
 *              这个参数由LVGL框架在调用回调函数时提供，用于标识和操作特定的显示设备。
 */
void lv_flush_done_cb(void* param)
{
    // 通知LVGL显示刷新已完成，可以准备下一帧的渲染。
    lv_disp_flush_ready(&disp_drv);
}

/**
 * @brief 初始化显示缓冲区和显示驱动
 * 
 * 该函数负责初始化用于LCD显示的缓冲区以及显示驱动结构体。它首先创建一个显示缓冲区，
 * 然后初始化显示驱动结构体，并注册到LVGL库中。这个过程包括分配缓冲区内存、初始化缓冲区、
 * 设置显示驱动的参数以及注册显示驱动。
 */
void lv_dis_init(void)
{
    // 创建一个静态显示缓冲区结构体
    static lv_disp_draw_buf_t  disp_buf;

    // 计算显示缓冲区的大小，这里假设缓冲区大小为LCD宽度乘以LCD高度的七分之一
    const size_t disp_buf_size = LCD_WIDTH * (LCD_HEIGHT/7);

    // 分配双缓冲区内存，使用DMA和内部内存分配
    lv_color_t *disp_buf_1 = heap_caps_malloc(disp_buf_size * sizeof(lv_color_t),MALLOC_CAP_INTERNAL|MALLOC_CAP_DMA);
    lv_color_t *disp_buf_2 = heap_caps_malloc(disp_buf_size * sizeof(lv_color_t),MALLOC_CAP_INTERNAL|MALLOC_CAP_DMA);

    // 检查缓冲区是否成功分配，如果分配失败，则释放已分配的内存并返回
    if(!disp_buf_1 || !disp_buf_2)
    {
        ESP_LOGE(TAG,"disp_buf_1 or disp_buf_2 malloc fail");
        lv_mem_free(disp_buf_1);
        lv_mem_free(disp_buf_2);
        return;
    }

    // 初始化显示缓冲区结构体
    lv_disp_draw_buf_init(&disp_buf,disp_buf_1,disp_buf_2,disp_buf_size);

    // 初始化显示驱动结构体
    lv_disp_drv_init(&disp_drv);

    // 设置显示驱动的参数，包括水平和垂直分辨率，以及刷新回调函数和绘制缓冲区
    disp_drv.hor_res = LCD_WIDTH;
    disp_drv.ver_res = LCD_HEIGHT;
    disp_drv.flush_cb = disp_flush;
    disp_drv.draw_buf = &disp_buf;

    // 注册显示驱动到LVGL库
    lv_disp_drv_register(&disp_drv);
}


/**
 * 初始化输入设备驱动
 * 
 * 该函数负责配置和注册一个输入设备驱动，以便LVGL图形库能够正确处理用户输入
 * 它使用LVGL提供的API来设置驱动的类型和读取回调函数
 */
void lv_indev_init(void)
{
    // 创建一个静态的输入设备驱动结构体
    static lv_indev_drv_t indev_drv;

    // 初始化输入设备驱动结构体
    lv_indev_drv_init(&indev_drv);

    // 设置输入设备的类型为指针类型，这通常用于触摸屏或鼠标
    indev_drv.type = LV_INDEV_TYPE_POINTER;

    // 设置读取输入设备数据的回调函数，这是LVGL与底层硬件交互的关键
    indev_drv.read_cb = indev_read;

    // 注册输入设备驱动，使其可以被LVGL使用
    lv_indev_drv_register(&indev_drv);
}
/**
 * @brief 初始化ST7789显示屏
 */
void st7789_hw_init(void)
{
    // 创建一个静态的ST7789显示驱动结构体
    static st7789_cfg_t st7789_cfg = {
        .bl = GPIO_NUM_26,
        .clk = GPIO_NUM_18,
        .cs = GPIO_NUM_5,
        .dc = GPIO_NUM_17,
        .mosi = GPIO_NUM_19,
        .rst = GPIO_NUM_21,
        .spi_fre = 40*1000*1000,
        .width = LCD_WIDTH,
        .height = LCD_HEIGHT,
        .done_cb = lv_flush_done_cb,
#if !LCDSHOWSTATE
        .spin = 1  // .spin = 1 表示旋转90度
#else
        .spin = 2 // 0默认不旋转，2旋转180
#endif
    };

    st7789_driver_hw_init(&st7789_cfg);
}
/**
 * @brief 初始化cst816t触摸输入
 */
void cst816t_hw_init()
{
    // 创建一个静态的CST816T触摸屏驱动结构体
    static cst816t_cfg_t cst816t_cfg = {
        .fre = 300*1000,
        .scl = GPIO_NUM_22,
        .sda = GPIO_NUM_23,
        .x_limit = LCD_WIDTH, 
        .y_limit = LCD_HEIGHT,
    };

    cst816t_init(&cst816t_cfg);
}


//定时器回调函数
void lv_timer_cb(void* arg)
{
    uint32_t tick_interval = *((uint32_t*)arg);
    lv_tick_inc(tick_interval);

}

/**
 * 初始化LVGL的tick计时器
 *
 * 该函数设置一个周期性硬件定时器，用于生成LVGL图形库所需的tick事件
 * 定时器的回调函数是lv_timer_cb，它将在每个tick间隔被调用
 */
void lv_tick_init(void)
{
    // 定义tick事件的间隔时间，单位为秒
    static uint32_t tick_interval = 5;
    
    // 定义定时器的配置参数
    const esp_timer_create_args_t arg = { 
        // 将tick_interval作为参数传递给回调函数
        .arg = &tick_interval,
        // 定义定时器的回调函
        .callback = lv_timer_cb,
        // 使用ESP_TIMER_TASK作为调度方法，意味着回调函数将在RTOS任务的上下文中运行
        .dispatch_method = ESP_TIMER_TASK,
        // 为定时器命名，便于调试和识别
        .name = "lvgl_tick",
        // 设置跳过未处理的事件，以避免回调函数被频繁调用时的累积执行延迟
        .skip_unhandled_events = true
    };

    // 定义定时器的句柄
    esp_timer_handle_t timer_handle;
    
    // 创建定时器
    esp_timer_create(&arg, &timer_handle);
    
    // 启动定时器，设置为周期性模式，每个tick_interval毫秒触发一次
    esp_timer_start_periodic(timer_handle, tick_interval * 1000);
}

/**
 * 初始化设备端口
 * 
 * 该函数负责初始化LVGL库和硬件设备，包括显示屏和触摸控制器
 * 它按顺序调用多个初始化函数，以确保所有必要的组件都准备好
 */
void lv_port_init(void)
{
    // 初始化LVGL库
    lv_init(); 
    
    // 初始化ST7789显示屏控制器
    st7789_hw_init();
    
    // 初始化CST816T触摸控制器
    cst816t_hw_init();
    
    // 初始化LVGL显示接口
    lv_dis_init();
    
    // 初始化LVGL输入设备接口
    lv_indev_init();
    
    // 初始化LVGL滴答接口
    lv_tick_init();
}
