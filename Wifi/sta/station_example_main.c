#include <string.h>
#include <sys/socket.h>
#include <netdb.h>
#include <unistd.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"

#define SERVER_IP "192.168.4.1"  // IP del AP ESP32
#define SERVER_PORT 3333
static const char *TAG = "TCP_SERVER";
static const char *TAG_STA = "WIFI_STA";
//static const char *ID = "sensor1";

struct packet{
	int64_t tiempo_envio;
	char id[8];
};

// Configuración Wifi
#define AP_SSID "ESP32_AP"
#define AP_PSWD "12345678"

// Mensajes TCP
#define RTT_RESPONSE "RTT Response"

void tcp_server_task(void *pvParameters){
	//char rx_buffer[sizeof(struct packet)];
	char rx_buffer[128];
	char addr_str[128];
	struct sockaddr_in dest_addr;
	int addr_family = AF_INET;
	int ip_protocol = IPPROTO_IP;
	struct timeval tiempo_recibo, tiempo_reenvio;

	int listen_sock = socket(addr_family, SOCK_STREAM, ip_protocol);
	if(listen_sock < 0) {
		ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
		vTaskDelete(NULL);
		return;
	}

	dest_addr.sin_addr.s_addr = htonl(INADDR_ANY);
	dest_addr.sin_family = AF_INET;
	dest_addr.sin_port  = htons(SERVER_PORT);

	bind(listen_sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
	listen(listen_sock, 1);

	ESP_LOGI(TAG, "Servidor TCP escuchando en el puerto %d", SERVER_PORT);

	while(1){
		struct sockaddr_in6 source_addr;
		socklen_t addr_len  = sizeof(source_addr);
		int sock = accept(listen_sock, (struct sockaddr *)&source_addr, &addr_len);
		if (sock < 0){
			ESP_LOGE(TAG, "Error al aceptar conexión: errno %d", errno);
			break;
		}

		inet_ntoa_r(((struct sockaddr_in *)&source_addr)->sin_addr, addr_str, sizeof(addr_str) - 1);
		ESP_LOGI(TAG, "Conexión aceptada de %s", addr_str);

		int len = recv(sock, rx_buffer, sizeof(rx_buffer) -1, 0);
		gettimeofday(&tiempo_recibo, NULL);
		if (len > 0){
			rx_buffer[len] = 0;
			ESP_LOGI(TAG, "Mensaje recibido: %s", rx_buffer);
		}
		int err = send(sock, RTT_RESPONSE, strlen(RTT_RESPONSE), 0);
		gettimeofday(&tiempo_reenvio, NULL);
		if(err<0){
			ESP_LOGE(TAG, "Error enviando: errno %d", errno);
		}
		ESP_LOGI(TAG, "%s enviado", RTT_RESPONSE);
		int64_t diferencia = ((int64_t)tiempo_reenvio.tv_sec*1e6 + (int64_t)tiempo_reenvio.tv_usec) - ((int64_t)tiempo_recibo.tv_sec*1e6 + (int64_t)tiempo_recibo.tv_usec);
		ESP_LOGI(TAG, "Enviando tiempo en servidor %lld", diferencia);
		err = send(sock, &diferencia, sizeof(diferencia), 0);
		if (err<0){
			ESP_LOGE(TAG, "Error al enviar el tiempo: errno %d", errno);
		}

		close(sock);
	}
	close(listen_sock);
	vTaskDelete(NULL);
}

void wifi_init_sta() {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = AP_SSID, 
            .password = AP_PSWD,
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG_STA, "Conectando al AP...");

    ESP_ERROR_CHECK(esp_wifi_connect());
}

void app_main() {
    ESP_ERROR_CHECK(nvs_flash_init());
    wifi_init_sta();
    vTaskDelay(pdMS_TO_TICKS(5000)); // Espera a que se conecte
    xTaskCreate(tcp_server_task, "tcp_server", 4096, NULL, 5, NULL);
}

