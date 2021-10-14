
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "ping.h"

#include "tcpip_adapter_types.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"
#include "lwip/dns.h"
#include "ping/ping_sock.h"
#include "../wifi_config.h"
#include "../AP_mode/AP_mode.h"

#define MAX_TIMEOUTS 5
#define PING_TAG     "PING"

static uint8_t timeout_count = 0;

void initialise_mdns(void)
{
    //initialize mDNS
    ESP_ERROR_CHECK(mdns_init());
    //set mDNS hostname (required if you want to advertise services)
    ESP_ERROR_CHECK(mdns_hostname_set(MED_BRACELET_MDNS_HOSTNAME));
    ESP_LOGI(
            PING_TAG, "mdns hostname set to: [%s]", MED_BRACELET_MDNS_HOSTNAME);
}
static void cmd_ping_on_ping_success(esp_ping_handle_t hdl, void *args)
{
    ESP_LOGW(PING_TAG, "cmd_ping_on_ping_success call_back");
    timeout_count = 0;
    uint8_t   ttl;
    uint16_t  seqno;
    uint32_t  elapsed_time, recv_len;
    uint32_t  recv_num, sent_num;
    ip_addr_t target_addr;
    esp_ping_get_profile(hdl, ESP_PING_PROF_SEQNO, &seqno, sizeof(seqno));
    esp_ping_get_profile(hdl, ESP_PING_PROF_TTL, &ttl, sizeof(ttl));
    esp_ping_get_profile(
            hdl, ESP_PING_PROF_IPADDR, &target_addr, sizeof(target_addr));
    esp_ping_get_profile(hdl, ESP_PING_PROF_SIZE, &recv_len, sizeof(recv_len));
    esp_ping_get_profile(
            hdl, ESP_PING_PROF_TIMEGAP, &elapsed_time, sizeof(elapsed_time));
    esp_ping_get_profile(
            hdl, ESP_PING_PROF_REQUEST, &sent_num, sizeof(sent_num));
    esp_ping_get_profile(hdl, ESP_PING_PROF_REPLY, &recv_num, sizeof(recv_num));

    ESP_LOGI(PING_TAG,
             "%d bytes from %s icmp_seq=%d ttl=%d time=%d ms",
             recv_len,
             inet_ntoa(target_addr.u_addr.ip4),
             seqno,
             ttl,
             elapsed_time);
    ESP_LOGI(PING_TAG,
             "Number of sent = %d, Number of received = %d",
             sent_num,
             recv_num);
}

static void cmd_ping_on_ping_timeout(esp_ping_handle_t hdl, void *args)
{
    ESP_LOGW(PING_TAG, "cmd_ping_on_ping_timeout call_back");
    timeout_count++;
    uint16_t  seqno;
    ip_addr_t target_addr;
    esp_ping_get_profile(hdl, ESP_PING_PROF_SEQNO, &seqno, sizeof(seqno));
    esp_ping_get_profile(
            hdl, ESP_PING_PROF_IPADDR, &target_addr, sizeof(target_addr));
    ESP_LOGW(PING_TAG,
             "From %s icmp_seq=%d timeout",
             inet_ntoa(target_addr.u_addr.ip4),
             seqno);
    if (timeout_count >= MAX_TIMEOUTS) {
        ESP_LOGW(PING_TAG,
                 "Maximum timeout pings number reached, reconfiguring to AP");
    }
}

static void cmd_ping_on_ping_end(esp_ping_handle_t hdl, void *args)
{
    ESP_LOGW(PING_TAG, "Ping end call_back");
    ip_addr_t target_addr;
    uint32_t  transmitted;
    uint32_t  received;
    uint32_t  total_time_ms;
    esp_ping_get_profile(
            hdl, ESP_PING_PROF_REQUEST, &transmitted, sizeof(transmitted));
    esp_ping_get_profile(hdl, ESP_PING_PROF_REPLY, &received, sizeof(received));
    esp_ping_get_profile(
            hdl, ESP_PING_PROF_IPADDR, &target_addr, sizeof(target_addr));
    esp_ping_get_profile(
            hdl, ESP_PING_PROF_DURATION, &total_time_ms, sizeof(total_time_ms));
    uint32_t loss = (uint32_t)((1 - ((float)received) / transmitted) * 100);
    if (IP_IS_V4(&target_addr)) {
        ESP_LOGI(PING_TAG,
                 "\n--- %s ping statistics ---",
                 inet_ntoa(*ip_2_ip4(&target_addr)));
    } else {
        ESP_LOGI(PING_TAG,
                 "\n--- %s ping statistics ---",
                 inet6_ntoa(*ip_2_ip6(&target_addr)));
    }
    ESP_LOGI(PING_TAG,
             "%d packets transmitted, %d received, %d%% packet loss, time %dms",
             transmitted,
             received,
             loss,
             total_time_ms);
    // delete the ping sessions, so that we clean up all resources and can create a new ping session
    // we don't have to call delete function in the callback, instead we can call delete function from other tasks
    esp_ping_delete_session(hdl);
}

/*
ping to target forever
interval_ms:ping interval mSec. Default is 1000mSec.
task_prio:ping task priority. Default is 2.
target_host:target host url. if null,target is own gateway.
*/
esp_err_t
initialize_ping(uint32_t interval_ms, uint32_t task_prio, char *target_host)
{
    esp_ping_config_t ping_config = ESP_PING_DEFAULT_CONFIG();

    if (strlen(target_host) > 0) {
        /* convert URL to IP address */
        ip_addr_t target_addr;
        memset(&target_addr, 0, sizeof(target_addr));
        struct addrinfo hint;
        memset(&hint, 0, sizeof(hint));
        struct addrinfo *res = NULL;
        int              err = getaddrinfo(target_host, NULL, &hint, &res);
        if (err != 0 || res == NULL) {
            ESP_LOGE(PING_TAG, "DNS lookup failed err=%d res=%p", err, res);
            return ESP_FAIL;
        } else {
            ESP_LOGI(PING_TAG, "DNS lookup success");
        }

        if (res->ai_family == AF_INET) {
            struct in_addr addr4 =
                    ((struct sockaddr_in *)(res->ai_addr))->sin_addr;
            inet_addr_to_ip4addr(ip_2_ip4(&target_addr), &addr4);
        } else {
            struct in6_addr addr6 =
                    ((struct sockaddr_in6 *)(res->ai_addr))->sin6_addr;
            inet6_addr_to_ip6addr(ip_2_ip6(&target_addr), &addr6);
        }
        freeaddrinfo(res);
        ESP_LOGI(PING_TAG, "target_addr.type=%d", target_addr.type);
        ESP_LOGI(PING_TAG,
                 "target_addr=%s",
                 ip4addr_ntoa(&(target_addr.u_addr.ip4)));
        ping_config.target_addr = target_addr; // target IP address
    } else {
        // ping target is my gateway
        tcpip_adapter_ip_info_t ip_info;
        ESP_ERROR_CHECK(
                tcpip_adapter_get_ip_info(TCPIP_ADAPTER_IF_STA, &ip_info));
        ESP_LOGI(PING_TAG, "IP Address: %s", ip4addr_ntoa(&ip_info.ip));
        ESP_LOGI(PING_TAG, "Subnet mask: %s", ip4addr_ntoa(&ip_info.netmask));
        ESP_LOGI(PING_TAG, "Gateway: %s", ip4addr_ntoa(&ip_info.gw));
        ip_addr_t gateway_addr;
        gateway_addr.type       = 0;
        gateway_addr.u_addr.ip4 = ip_info.gw;
        ESP_LOGI(PING_TAG, "gateway_addr.type=%d", gateway_addr.type);
        ESP_LOGI(PING_TAG,
                 "gateway_addr=%s",
                 ip4addr_ntoa(&(gateway_addr.u_addr.ip4)));
        ping_config.target_addr = gateway_addr; // gateway IP address
    }

    ping_config.count =
            ESP_PING_COUNT_INFINITE; // ping in infinite mode, esp_ping_stop can stop it
    ping_config.interval_ms = interval_ms;
    ping_config.task_prio   = task_prio;
    /* Stack size incresed. Reason - after reconfiguration to AP, ping task stackoverflow with standrd size */
    ping_config.task_stack_size = PING_TASK_STACK_SIZE;

    /* set callback functions */
    esp_ping_callbacks_t cbs = { .on_ping_success = cmd_ping_on_ping_success,
                                 .on_ping_timeout = cmd_ping_on_ping_timeout,
                                 .on_ping_end     = cmd_ping_on_ping_end,
                                 .cb_args         = NULL };
    esp_ping_handle_t    ping;
    esp_ping_new_session(&ping_config, &cbs, &ping);
    esp_ping_start(ping);
    ESP_LOGI(PING_TAG, "ping start");
    return ESP_OK;
}
