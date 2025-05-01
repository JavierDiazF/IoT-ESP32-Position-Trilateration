#include <string.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include <math.h>

#define SERVER_PORT 3333
static const char *TAG = "TCP_CLIENT";
static const char *TAG_IP = "IP_EVENT";
static const char *TAG_AP = "WIFI_AP";
static const char *TAG_DIST = "POSICION";

struct packet{
	int64_t tiempo_server;
	char id[8];
	double coordx;
	double coordy;
};
// Configuración AP
#define AP_SSID "ESP32_AP"
#define AP_PSWD "12345678"

//Número máximo de clientes
#define MAX_CLIENTES 4
static esp_ip4_addr_t ip_clientes[MAX_CLIENTES];
static int num_clientes = 0;
static float apcoordx, apcoordy, coordenadas[MAX_CLIENTES][3]; // La estructura es coordx, coordy, distancia
// Mensajes TCP
#define RTT_QUERY "RTT Query"

void calcula_posicion_task(void *pvParameters){
	float A, B, C, D, E, F, denominador;
	A = 2*(coordenadas[0][0] - coordenadas[1][0]); //2*(x_1 - x_2)
	B = 2*(coordenadas[0][1] - coordenadas[1][1]); //2*(y_1 - y_2)
	C = pow(coordenadas[0][2],2) - pow(coordenadas[1][2],2) - pow(coordenadas[0][0],2) + pow(coordenadas[1][0],2) - pow(coordenadas[0][1],2) + pow(coordenadas[1][1],2);
	
	D = 2*(coordenadas[0][0] - coordenadas[2][0]); //2*(x_1 - x_3)
	E = 2*(coordenadas[0][1] - coordenadas[2][1]); //2*(y_1 - y_3)
	F = pow(coordenadas[0][2],2) - pow(coordenadas[2][2],2) - pow(coordenadas[0][0],2) + pow(coordenadas[2][0],2) - pow(coordenadas[0][1],2) + pow(coordenadas[2][1],2);
	denominador = (A*E-B*D);
	if(fabs(denominador)< 1e-6){
		ESP_LOGE(TAG_DIST, "No se puede resolver (división por cero)");
		vTaskDelete(NULL);
		return;
	}
	apcoordx = (C*E-F*B)/denominador;
	apcoordy = (A*F-D*C)/denominador;

	vTaskDelete(NULL);
}
void tcp_client_task(void *pvParameters){
	esp_ip4_addr_t server_ip = ip_clientes[num_clientes-1];
	struct sockaddr_in dest_addr;
	dest_addr.sin_addr.s_addr = server_ip.addr;
	dest_addr.sin_family = AF_INET;
	dest_addr.sin_port = htons(SERVER_PORT);

	// Para medir el RTT hacemos un timestamp al enviar y  otro al recibir el mensaje
	struct timeval tiempo_envio, tiempo_recibo;

	char rx_buffer[sizeof(struct packet)];
	struct packet *rx_pkt = (struct packet*) rx_buffer;
	
	int sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
	if (sock < 0){
		ESP_LOGE(TAG, "No se pudo crear socket: errno %d", errno);
		vTaskDelete(NULL);
		return;
	}
	ESP_LOGI(TAG, "Conectando al servidor " IPSTR ":%d...", IP2STR(&server_ip), SERVER_PORT);
	if (connect(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr))!=0){
		ESP_LOGE(TAG, "Fallo en la conexíon: errno %d", errno);
		close(sock);
		vTaskDelete(NULL);
		return;
	}
	// Mandamos el RTT Query
	//gettimeofday(&tiempo_envio, NULL);
	int err  = send(sock, RTT_QUERY, strlen(RTT_QUERY), 0);
	gettimeofday(&tiempo_envio, NULL);
	if (err<0){
		ESP_LOGE(TAG, "Error enviando: errno %d", errno);
	}
	ESP_LOGI(TAG, "RTT Query enviado. Esperando respuesta...");
	int len = recv(sock, rx_buffer, sizeof(rx_buffer)-1, 0);
	gettimeofday(&tiempo_recibo, NULL);
	if (len>0){
		rx_buffer[len] = 0;
		ESP_LOGI(TAG, "Mensaje recibido: %s", rx_buffer);
		if(strncmp("RTT Response", rx_buffer, strlen("RTT Response")-1) == 0){
			int64_t tiempo_total = ((int64_t)tiempo_recibo.tv_sec*1e6 + (int64_t)tiempo_recibo.tv_usec) - ((int64_t)tiempo_envio.tv_sec*1e6 + (int64_t)tiempo_envio.tv_usec);
			// Recibimos el tiempo en otro paquete
			len = recv(sock, &rx_buffer, sizeof(rx_buffer), 0);
			if (len>0){
				int64_t diferencia = ((int64_t)tiempo_recibo.tv_sec*1e6 + (int64_t)tiempo_recibo.tv_usec) - ((int64_t)tiempo_envio.tv_sec*1e6 + (int64_t)tiempo_envio.tv_usec) - rx_pkt->tiempo_server;
				double distancia = 3e8*diferencia/(2*1e6);
				ESP_LOGI(TAG, "Tiempo total: %lld; Tiempo en servidor: %lld", tiempo_total, rx_pkt->tiempo_server);
				ESP_LOGI(TAG, "Tiempo: %lld Distancia %g m, sensor: %s", diferencia, distancia, rx_pkt->id);
				coordenadas[num_clientes][0] = rx_pkt->coordx;
				coordenadas[num_clientes][1] = rx_pkt->coordy;
				coordenadas[num_clientes][2] = distancia;
				if(num_clientes == 1)
					xTaskCreate(calcula_posicion_task, "tcp_client", 4096, NULL, 5, NULL);
				else
					ESP_LOGI("N_clientes", "Se ha guardado cliente %d", num_clientes);
			}
		}

	} else {
		ESP_LOGE(TAG, "Error en la recepción");
	}

	close(sock);
	vTaskDelete(NULL);
}

static void ip_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data){
	if(event_id == IP_EVENT_AP_STAIPASSIGNED){
		ip_event_ap_staipassigned_t *event = (ip_event_ap_staipassigned_t*) event_data;
		if (num_clientes < MAX_CLIENTES){
			ip_clientes[num_clientes] = event->ip;
			ESP_LOGI(TAG_IP, "Se ha conectado una nueva estación con IP " IPSTR, IP2STR(&event->ip));
			//xTaskCreate(tcp_client_task, "tcp_client", 4096, event->ip, 5, NULL);
			vTaskDelay(pdMS_TO_TICKS(5000)); // Espero a que se conecte el sta
			xTaskCreate(tcp_client_task, "tcp_client", 4096, NULL, 5, NULL);
			num_clientes++; // Una vez terminado todo actualizamos el número de clientes
		} else {
			ESP_LOGE(TAG_IP, "Número máximo de clientes alcanzado");
		}
	}
}
void init_wifi_ap() {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_AP_STAIPASSIGNED, &ip_event_handler, NULL, NULL));

    wifi_config_t ap_config = {
        .ap = {
            .ssid = AP_SSID,
            .ssid_len = 0,
            .password = AP_PSWD,
            .max_connection = MAX_CLIENTES,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG_AP, "AP inicializado. SSID: %s, password: %s", AP_SSID, AP_PSWD);
}

void app_main() {
    ESP_ERROR_CHECK(nvs_flash_init());
    init_wifi_ap();
}

