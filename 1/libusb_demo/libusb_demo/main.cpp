#include <libusb.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

char tx_buf[128];

#define vendor_id   0x0483
#define product_id  0x5740

int next = 0;

libusb_device_handle * handle = NULL;


float pitch = 0.0f;
float roll  = 0.0f;
float yaw   = 0.0f;


void LIBUSB_CALL bulk_cb(struct libusb_transfer *transfer)
{
    if (transfer->status == LIBUSB_TRANSFER_COMPLETED) {
        printf("Received %d bytes\n", transfer->actual_length);

        // 处理数据
        for (int i = 0; i < transfer->actual_length; i++) {
            printf("%02X ", transfer->buffer[i]);
        }
        printf("\n");

		printf("get gyro data---------------------------\r\n");
		
		
	if( transfer->actual_length == 12 )
	{
		printf("\033[10;20H"); // move to: row 10 column 20
	
		unsigned char* rx_buf = transfer->buffer;
	
		int16_t data = rx_buf[6];
		data <<= 8;
		data += rx_buf[5];
		
		roll = data;
		roll /= 10.0;
		printf("roll = %3.3f  ", roll);
		
		data = rx_buf[8];
		data <<= 8;
		data += rx_buf[7];
		pitch = data;
		pitch /= 10.0;
		printf("pitch = %3.3f  ", pitch);


		data = rx_buf[10];
		data <<= 8;
		data += rx_buf[9];
		yaw = data;
		yaw /= 10.0;
		printf("yaw = %3.3f  ", yaw);
		
		printf("\r\n\r\n");


		fflush(stdout);
	}
		
		next = 1;
        // 🔁 关键：重新提交，实现持续监听
        libusb_submit_transfer(transfer);
    }
    else {
        printf("Transfer error: %d\n", transfer->status);
        libusb_free_transfer(transfer);
    }
}


const char* libusb_transfer_status_desc(enum libusb_transfer_status status)
{
    switch (status) {
    case LIBUSB_TRANSFER_COMPLETED:
        return "Transfer completed successfully";
    case LIBUSB_TRANSFER_ERROR:
        return "Transfer failed";
    case LIBUSB_TRANSFER_TIMED_OUT:
        return "Transfer timed out";
    case LIBUSB_TRANSFER_CANCELLED:
        return "Transfer was cancelled";
    case LIBUSB_TRANSFER_STALL:
        return "Endpoint stalled (halt condition)";
    case LIBUSB_TRANSFER_NO_DEVICE:
        return "Device disconnected";
    case LIBUSB_TRANSFER_OVERFLOW:
        return "Device sent more data than requested";
    default:
        return "Unknown transfer status";
    }
}

int send_data(libusb_device_handle *h, const uint8_t *data, int len) {
    int actual;
    int ret = libusb_bulk_transfer(
        h,                    // 设备句柄
        0x01,                 // 目标端点（OUT方向）
        (unsigned char*)data, // 数据缓冲区
        len,                  // 长度
        &actual,              // 实际传输长度
        1000                  // 超时1秒
    );

    if (ret == 0) {
        printf("发送成功：%d 字节\n", actual);
        return 0;
    } else {
        fprintf(stderr, "发送失败：%s\n", libusb_error_name(ret));
        return -1;
    }
}

int send_data_req_state(void)
{
    tx_buf[0] = 0x24;
    tx_buf[1] = 0x4d;
    tx_buf[2] = 0x3c;
    tx_buf[3] = 0x00;
    tx_buf[4] = 0x6c;
    tx_buf[5] = 0x6c;
    tx_buf[6] = '\0';
    int write_len = 6;
    
    send_data(handle,(unsigned char*) tx_buf, write_len );
    
    return 0;
}


#define BUF_SIZE 64
#define NUM_BUFS 3

libusb_transfer *transfers[NUM_BUFS];

void LIBUSB_CALL interrupt_callback(struct libusb_transfer *t) {
    if (t->status == LIBUSB_TRANSFER_COMPLETED) {
        printf("收到中断数据：%.*s\n", t->actual_length, (char*)t->buffer);

        // 关键：重新提交，形成循环队列
        libusb_submit_transfer(t);
    } else {
        fprintf(stderr, "中断错误：%s\n", libusb_transfer_status_desc(t->status));
    }
}

void start_interrupt_reads(libusb_device_handle *handle, uint8_t ep) {
    for (int i = 0; i < NUM_BUFS; i++) {
        transfers[i] = libusb_alloc_transfer(0);
        unsigned char *buf = (unsigned char*)malloc(BUF_SIZE);

        libusb_fill_interrupt_transfer(
            transfers[i],     // 传输结构
            handle,           // 句柄
            ep,               // 端点（IN方向，如0x82）
            buf,              // 缓冲区
            BUF_SIZE,
            interrupt_callback,
            NULL,             // 用户数据
            500               // 超时ms
        );

        libusb_submit_transfer(transfers[i]);
    }
}

libusb_hotplug_callback_handle cb_handle;

int LIBUSB_CALL hotplug_callback(libusb_context *ctx, libusb_device *dev,
                                 libusb_hotplug_event event, void *user_data) {
    if (event == LIBUSB_HOTPLUG_EVENT_DEVICE_ARRIVED) {
        printf("设备插入！\n");
        // 这里可以尝试打开并初始化
        
    	//  查找并打开设备
    	handle = libusb_open_device_with_vid_pid(ctx, vendor_id, product_id);
    	if (!handle) {
        	fprintf(stderr, "找不到设备 (%04x:%04x)\n", vendor_id, product_id);
        	libusb_exit(NULL);
        	return -1;
    	}
	
    	printf("成功打开设备！\n");
	
    	//  分离可能存在的内核驱动（Linux常见）
    	if (libusb_kernel_driver_active(handle, 0)) {
        	libusb_detach_kernel_driver(handle, 0);
    	}
	
    	//  占用接口0
    	int ret = libusb_claim_interface(handle, 0);
    	if (ret != 0) 
    	{
    		printf("libusb claim interface error\r\n");
    		return ret;
    	}
    	else
    	{
    		printf("libusb claim interface success\r\n");
    	}
        	
    } else if (event == LIBUSB_HOTPLUG_EVENT_DEVICE_LEFT) {
        printf("设备已拔出\n");
        // 标记状态、停止读写线程
    }
    printf("usb hotplug return\r\n");
    return 0;
}


int main() {
    libusb_device **list;
	libusb_context *ctx = NULL;
    ssize_t cnt;

    // 初始化上下文
    if (libusb_init(&ctx) != 0) {
        fprintf(stderr, "初始化失败\n");
        return -1;
    }

    // 可选：开启调试日志
    libusb_set_debug(ctx, 3);  // 日志级别 3=详细

    // 获取当前连接的所有USB设备
    cnt = libusb_get_device_list(ctx, &list);
    if (cnt < 0) {
        fprintf(stderr, "获取设备列表失败\n");
        libusb_exit(ctx);
        return -1;
    }

    printf("共发现 %zd 个设备:\n", cnt);

    for (ssize_t i = 0; i < cnt; i++) {
        libusb_device_descriptor desc;
        libusb_get_device_descriptor(list[i], &desc);

        printf("  [%zd] VID:%04x PID:%04x",
               i, desc.idVendor, desc.idProduct);

        // 打印设备速度
        switch (libusb_get_device_speed(list[i])) {
            case LIBUSB_SPEED_LOW:   printf(" [低速]"); break;
            case LIBUSB_SPEED_FULL:  printf(" [全速]"); break;
            case LIBUSB_SPEED_HIGH:  printf(" [高速]"); break;
            case LIBUSB_SPEED_SUPER: printf(" [超速]"); break;
            default: printf(" [未知]");
        }
        printf("\n");
    }


	

// 注册监听
	libusb_hotplug_register_callback(ctx,
    	LIBUSB_HOTPLUG_EVENT_DEVICE_ARRIVED | LIBUSB_HOTPLUG_EVENT_DEVICE_LEFT,
    	LIBUSB_HOTPLUG_ENUMERATE,
    	vendor_id, product_id, LIBUSB_HOTPLUG_MATCH_ANY,
    	hotplug_callback, NULL, &cb_handle);
	
//	start_interrupt_reads(handle, 0x81 );


	while(!handle  )
	{
		static int counter = 0;
		printf("\033[10;20H"); // move to: row 10 column 20

		printf("waiting for usb ready...%d\r\n", counter++);
		usleep(20000);
	}

	printf("start bulk cb...\r\n");
	
	unsigned char *buf =(unsigned char*)malloc(512);
	
	struct libusb_transfer *transfer = libusb_alloc_transfer(0);
	
	libusb_fill_bulk_transfer(
    	transfer,
    	handle,
    	0x81,                  // endpoint (IN)
    	buf,
    	512,
    	bulk_cb,
    	NULL,
    	0                      // timeout (0 = infinite)
	);
	
	libusb_submit_transfer(transfer);

	printf("bulk cb...ready\r\n");


	bool running = true;
	while (running) {
		next = 0;
    	printf("request state data\r\n");
		send_data_req_state();    	
    	libusb_handle_events(ctx);  // 处理异步回调
/*		
		unsigned char in_data[64];
    	int actual_length;
    
		int ret = libusb_bulk_transfer(handle, 0x81, in_data, sizeof(in_data), &actual_length, 1000);
    	if (ret == 0) {
        	printf("收到 %d 字节: ", actual_length);
        	for (int i = 0; i < actual_length; ++i)
            	printf("%02x ", in_data[i]);
        	printf("\n");
    	}
	*/
	
		while(!next){};	
			
//    	usleep(1000000);               // 避免CPU空转
	}
	


    // 清理资源
    libusb_free_device_list(list, 1);  // 第二个参数为1表示自动释放每个设备
    libusb_exit(ctx);
    return 0;
}


/*
libusb 的异步模型依赖一个“事件泵”机制。如果你不在主线程调用 libusb_handle_events() ，哪怕数据到了，回调也不会执行。

解决方案：

单线程应用：主循环里定期调用 libusb_handle_events_timeout() 。
多线程：启动专用线程运行 libusb_handle_events() 。
使用 libusb_hotplug_register_callback() 监听设备插拔，也需要事件循环支持。
*/
