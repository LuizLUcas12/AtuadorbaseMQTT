#include "EspMQTTClient.h"       // Inclui a biblioteca EspMQTTClient para gerenciar a conexão MQTT.
#include <ArduinoJson.h>         // Inclui a biblioteca ArduinoJson para manipulação de JSON.
#include "ConnectDataIFRN.h"     // Inclui um arquivo de cabeçalho personalizado que provavelmente contém informações de configuração como credenciais de Wi-Fi e MQTT.

#define DATA_INTERVAL 10000       // Define o intervalo de tempo para adquirir novos dados do sensor (em milissegundos).
#define AVAILABLE_INTERVAL 5000   // Define o intervalo de tempo para enviar sinais de disponibilidade (em milissegundos).
#define LED_INTERVAL_MQTT 1000    // Define o intervalo de tempo para piscar o LED quando conectado ao broker MQTT.
#define JANELA_FILTRO 1           // Define o número de amostras do filtro para realizar a média (não utilizado neste código).

byte VALVULA1_PIN = 12;           // Define o pino GPIO 12 para controle da válvula 1.


unsigned long dataIntevalPrevTime = 0;      // Armazena o tempo da última aquisição de dados.
unsigned long availableIntevalPrevTime = 0; // Armazena o tempo do último envio de sinal de disponibilidade.

void setup()
{
  Serial.begin(115200);           // Inicia a comunicação serial a 115200 bps.

  pinMode(LED_BUILTIN, OUTPUT);   // Configura o pino do LED integrado como saída.
  pinMode(VALVULA1_PIN, OUTPUT);  // Configura o pino da válvula 1 como saída.


  // Funcionalidades opcionais do EspMQTTClient
  //client.enableMQTTPersistence();             // Opcional: habilita a persistência MQTT (comentado no código).
  client.enableDebuggingMessages();           // Habilita a exibição de mensagens de depuração na saída serial.
  client.enableHTTPWebUpdater();              // Habilita o atualizador web (para atualizações OTA).
  client.enableOTA();                         // Habilita atualizações Over The Air (OTA).
  client.enableLastWillMessage(TOPIC_AVAILABLE , "offline");  // Define uma mensagem de "last will" que será enviada se o dispositivo desconectar inesperadamente.

  WiFi.mode(WIFI_STA);            // Configura o Wi-Fi para operar no modo Station (conectando-se a uma rede existente).
}

void valvula1(const String payload) {
  // Função chamada quando uma mensagem é recebida no tópico da válvula 1.
  if (payload == "ON") {
    digitalWrite(VALVULA1_PIN, HIGH); // Liga a válvula 1.
  }
  else {
    digitalWrite(VALVULA1_PIN, LOW);  // Desliga a válvula 1.
  }
 
  client.executeDelayed(1 * 100, metodoPublisher); // Executa a função metodoPublisher após 100 ms.
}



void onConnectionEstablished()
{
  // Função chamada quando a conexão com o broker MQTT é estabelecida.
  client.subscribe(topic_name + "/valv1/set", valvula1); // Inscreve-se no tópico para controle da válvula 1.


  availableSignal(); // Envia sinal de disponibilidade ("online").
}

void availableSignal() {
  // Publica uma mensagem no tópico de disponibilidade indicando que o dispositivo está online.
  client.publish(TOPIC_AVAILABLE, "online");
}

void metodoPublisher() {
  // Função responsável por criar um JSON com os dados do dispositivo e publicá-lo no broker MQTT.

  StaticJsonDocument<300> jsonDoc;  // Cria um documento JSON com tamanho máximo de 300 bytes.

  // Adiciona dados ao documento JSON.
  jsonDoc["RSSI"]     = WiFi.RSSI();  // Intensidade do sinal Wi-Fi.
  jsonDoc["valvula1"] = digitalRead(VALVULA1_PIN)==1 ? "ON" : "OFF";  // Estado da válvula 1.


  String payload = "";                // String que armazenará o JSON serializado.
  serializeJson(jsonDoc, payload);    // Serializa o JSON para a string.

  client.publish(topic_name, payload); // Publica o payload JSON no tópico MQTT.
}

void blinkLed() {
  // Função que controla o LED integrado do dispositivo para indicar o estado da conexão.

  static unsigned long ledWifiPrevTime = 0;  // Armazena o tempo da última mudança do LED para Wi-Fi.
  static unsigned long ledMqttPrevTime = 0;  // Armazena o tempo da última mudança do LED para MQTT.
  unsigned long time_ms = millis();          // Obtém o tempo atual em milissegundos.
  bool ledStatus = false;                    // Estado atual do LED.

  if ( (WiFi.status() == WL_CONNECTED)) {    // Se o Wi-Fi estiver conectado...
    if (client.isMqttConnected()) {          // Se o cliente MQTT também estiver conectado...
      if ( (time_ms - ledMqttPrevTime) >= LED_INTERVAL_MQTT) { // Se o tempo decorrido desde a última mudança for maior que o intervalo definido...
        ledStatus = !digitalRead(LED_BUILTIN);  // Inverte o estado do LED.
        digitalWrite(LED_BUILTIN, ledStatus);   // Atualiza o LED.
        ledMqttPrevTime = time_ms;              // Atualiza o tempo da última mudança.
      }
    }
    else {
      digitalWrite(LED_BUILTIN, LOW); // Se o MQTT não estiver conectado, liga o LED (estado fixo).
    }
  }
  else {
    digitalWrite(LED_BUILTIN, HIGH); // Se o Wi-Fi não estiver conectado, desliga o LED.
  }
}

void loop()
{
  // Função principal que é executada continuamente.

  unsigned long time_ms = millis(); // Obtém o tempo atual em milissegundos.

  client.loop();  // Mantém a conexão MQTT ativa.

  if (time_ms - dataIntevalPrevTime >= DATA_INTERVAL) {
    // Se o tempo decorrido desde a última aquisição de dados for maior que o intervalo definido...
    client.executeDelayed(1 * 100, metodoPublisher); // Executa a função metodoPublisher após 100 ms.
    dataIntevalPrevTime = time_ms; // Atualiza o tempo da última aquisição de dados.
  }

  if (time_ms - availableIntevalPrevTime >= AVAILABLE_INTERVAL) {
    // Se o tempo decorrido desde o último sinal de disponibilidade for maior que o intervalo definido...
    client.executeDelayed(1 * 500, availableSignal); // Executa a função availableSignal após 500 ms.
    availableIntevalPrevTime = time_ms; // Atualiza o tempo do último envio de sinal de disponibilidade.
  }

  blinkLed();  // Chama a função que controla o LED.
}
