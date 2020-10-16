#include "memory"
#include <string.h>
#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event_loop.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_wifi_types.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "driver/uart.h"
//#include "rom/uart.h"
//#include "esp8266/rom_functions.h"
//#include "internal/esp_system_internal.h"
#include "AsyncTCP.h"
#include "Util.h"

/* The examples use simple WiFi configuration that you can set via
   'make menuconfig'.
   If you'd rather not, just change the below entries to strings with
   the config you want - ie #define EXAMPLE_WIFI_SSID "mywifissid"
*/
#define EXAMPLE_WIFI_SSID CONFIG_WIFI_SSID
#define EXAMPLE_WIFI_PASS CONFIG_WIFI_PASSWORD

/* FreeRTOS event group to signal when we are connected & ready to make a request */
static EventGroupHandle_t wifi_event_group;

//NetPP::Socket listener(NetPP::Socket(NetPP::IPVersion::IPv4));



const int IPV4_GOTIP_BIT = BIT0;
#ifdef CONFIG_EXAMPLE_IPV6
const int IPV6_GOTIP_BIT = BIT1;
#endif

static const char *TAG = "example";


//static std::vector<AsyncClient*> clients;       // a list to hold all clients

 /* clients events */
static void handleError(void* arg, AsyncClient* client, int8_t error) {
	ESP_LOGI(TAG, "connection error %s from client %s \n", client->errorToString(error), client->remoteIP().toString().c_str());
}

static void handleData(void* arg, AsyncClient* client, void *data, size_t len) {
	ESP_LOGI(TAG, "data received from client %s \n", client->remoteIP().toString().c_str());
	ESP_LOGI(TAG, "%.*s\n", len,(const char*)data);

	char reply[64];
	// reply to client
	if(client->space() > sizeof(reply) && client->canSend()) {
		
		sprintf(reply, "this is from %s", "esp8266_server");
		client->add(reply, strlen(reply));
		client->send();
	}
}

static void handleDisconnect(void* arg, AsyncClient* client) {
	ESP_LOGI(TAG, "\n client %s disconnected \n", client->remoteIP().toString().c_str());
	delete client;
}

static void handleTimeOut(void* arg, AsyncClient* client, uint32_t time) {
	ESP_LOGI(TAG, "\n client ACK timeout ip: %s \n", client->remoteIP().toString().c_str());
}


/* server events */
static void handleNewClient(void* arg, AsyncClient* client) {
	ESP_LOGI(TAG, "\n new client has been connected to server, ip: %s", client->remoteIP().toString().c_str());
	
	// register events
	client->onData(&handleData, NULL);
	client->onError(&handleError, NULL);
	client->onDisconnect(&handleDisconnect, NULL);
	client->onTimeout(&handleTimeOut, NULL);
}

static esp_err_t event_handler(void *ctx, system_event_t *event)
{
	/* For accessing reason codes in case of disconnection */
	system_event_info_t *info = &event->event_info;

	switch (event->event_id) {
	case SYSTEM_EVENT_STA_START:
		esp_wifi_connect();
		ESP_LOGI(TAG, "SYSTEM_EVENT_STA_START");
		break;
	case SYSTEM_EVENT_STA_CONNECTED:
#ifdef CONFIG_EXAMPLE_IPV6
		/* enable ipv6 */
		tcpip_adapter_create_ip6_linklocal(TCPIP_ADAPTER_IF_STA);
#endif
		break;
	case SYSTEM_EVENT_STA_GOT_IP:
		xEventGroupSetBits(wifi_event_group, IPV4_GOTIP_BIT);
		ESP_LOGI(TAG, "SYSTEM_EVENT_STA_GOT_IP");
		break;
	case SYSTEM_EVENT_STA_DISCONNECTED:
		ESP_LOGE(TAG, "Disconnect reason : %d", info->disconnected.reason);
		if (info->disconnected.reason == WIFI_REASON_BASIC_RATE_NOT_SUPPORT) {
					
			/*Switch to 802.11 bgn mode */
			esp_wifi_set_protocol(ESP_IF_WIFI_STA, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11B);
		}
		esp_wifi_connect();
		xEventGroupClearBits(wifi_event_group, IPV4_GOTIP_BIT);
#ifdef CONFIG_EXAMPLE_IPV6
		xEventGroupClearBits(wifi_event_group, IPV6_GOTIP_BIT);
#endif
		break;
	case SYSTEM_EVENT_AP_STA_GOT_IP6:
#ifdef CONFIG_EXAMPLE_IPV6
		xEventGroupSetBits(wifi_event_group, IPV6_GOTIP_BIT);
		ESP_LOGI(TAG, "SYSTEM_EVENT_STA_GOT_IP6");

		char *ip6 = ip6addr_ntoa(&event->event_info.got_ip6.ip6_info.ip);
		ESP_LOGI(TAG, "IPv6: %s", ip6);
#endif
	default:
		break;
	}
	return ESP_OK;
}

static void initialise_wifi(void)
{
	tcpip_adapter_init();
	wifi_event_group = xEventGroupCreate();
	ESP_ERROR_CHECK(esp_event_loop_init(event_handler, NULL));
	wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
	
	ESP_ERROR_CHECK(esp_wifi_init(&cfg));
	//ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
	wifi_config_t wifi_config = {};
	
	
	
	memcpy(&wifi_config.sta.ssid, EXAMPLE_WIFI_SSID, sizeof(EXAMPLE_WIFI_SSID));
	memcpy(&wifi_config.sta.password, EXAMPLE_WIFI_PASS, sizeof(EXAMPLE_WIFI_PASS));
	
	
	ESP_LOGI(TAG, "Setting WiFi configuration SSID %s...", wifi_config.sta.ssid);
	ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
	ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
	ESP_ERROR_CHECK(esp_wifi_start());
}

static void wait_for_ip()
{
#ifdef CONFIG_EXAMPLE_IPV6
	uint32_t bits = IPV4_GOTIP_BIT | IPV6_GOTIP_BIT;
#else
	uint32_t bits = IPV4_GOTIP_BIT;
#endif

	ESP_LOGI(TAG, "Waiting for AP connection...");
	xEventGroupWaitBits(wifi_event_group, bits, false, true, portMAX_DELAY);
	ESP_LOGI(TAG, "Connected to AP");
}

static void heap_monitor_task(void *pvParameters)
{
	while (true)
	{
		ESP_LOGI(TAG, "FREE HEAP:%d\n", esp_get_free_heap_size());
		ESP_LOGI(TAG, "CONT. FREE HEAP:%d\n", esp_get_minimum_free_heap_size());
		vTaskDelay(500);
	}
}


extern "C" void app_main()
{
	/*
	 * idk why they call it in 'examples' when it's 
	 * already being called in 'user_init_entry' function
	 **/
	//ESP_ERROR_CHECK(nvs_flash_init()); 
	
 	
	initialise_wifi();
	wait_for_ip();
	
	gpio_config_t io_conf;
	io_conf.intr_type = GPIO_INTR_DISABLE;
	io_conf.mode = GPIO_MODE_OUTPUT;
	io_conf.pin_bit_mask = (1 << GPIO_NUM_2) | (1 << GPIO_NUM_16);
	io_conf.pull_down_en = gpio_pulldown_t::GPIO_PULLDOWN_DISABLE;
	io_conf.pull_up_en = gpio_pullup_t::GPIO_PULLUP_DISABLE;
	// configure GPIO with the given settings
	gpio_config(&io_conf);
	
	gpio_set_level(GPIO_NUM_2, 0);
	gpio_set_level(GPIO_NUM_16, 0);
	
	uart_config_t uart_config = {
		.baud_rate = 115200,
		.data_bits = UART_DATA_8_BITS,
		.parity = UART_PARITY_DISABLE,
		.stop_bits = UART_STOP_BITS_1,
		.flow_ctrl = UART_HW_FLOWCTRL_DISABLE
	};
	uart_param_config(UART_NUM_0, &uart_config);
	uart_driver_install(UART_NUM_0, 1024 * 2, 0, 0, NULL,0);
	
	
		
	/*
	static auto timer_now_us = micros();
	c.onConnect([](void* timer_now_us, AsyncClient* client) {
		ESP_LOGI(TAG, "took %lu\n", micros() -  (*reinterpret_cast<unsigned long*>(timer_now_us)));
	},
	&timer_now_us);
	
	c.onData([](void* arg, AsyncClient* client, void *data, size_t len) {
			ESP_LOGI(TAG, "received data with length: %u\n",len);
		},
		nullptr);
	c.connect("192.168.1.4", 56389);
	*/
	
	int8_t power = 0;
	ESP_ERROR_CHECK(esp_wifi_get_max_tx_power(&power));
	ESP_LOGI(TAG, "Wifi max tx power: %d\n", power);
	
	constexpr auto TCP_PORT = 48888;
	constexpr auto BACK_LOG = 20;

	AsyncServer* server = new AsyncServer(TCP_PORT);        // start listening on tcp port 7050
	server->onClient(&handleNewClient, server);
	server->begin(BACK_LOG);
	//xTaskCreate(tcp_server_task, "tcp_server_task", 4096, NULL, 5, NULL);
	xTaskCreate(heap_monitor_task, "heap_monitor_task", 1024, NULL, 4, NULL);
	
}
