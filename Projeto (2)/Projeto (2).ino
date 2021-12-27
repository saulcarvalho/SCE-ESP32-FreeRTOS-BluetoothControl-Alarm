/*------------------------------------------------------------------------------\
  | Projeto Final para UC Sistemas Computacionais Embedidos                       |
  | Aluno 1: Saúl Carvalho        | Número: 2191748                               |
  | Aluno 2: Cristiano Moreira    | Número: 2191912                               |
  \------------------------------------------------------------------------------*/
#include <stdio.h>
#include "Adafruit_BusIO_Register.h"
#include "Adafruit_ILI9341.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_task_wdt.h"

/* --- Pinout do LCD (320x240) --- */    // LED + LCD -> alimentação 3.3V
#define TFT_MISO 19 // SD0
#define TFT_SCK 18  // Clock
#define TFT_MOSI 23 // SDI
#define TFT_DC 0    // 
#define TFT_RESET 2 // RESET
#define TFT_CS 15   // Chip Select
Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCK, TFT_RESET, TFT_MISO);

/* --- Pinout do Joystick --- */
#define SW 33   // Pressionado -> 0;  Default -> 4095;
#define X 25    // 0 - 4095   (Centro ~ 1890)
#define Y 26    // 0 - 4095   (Centro ~ 1845)

/* --- Pinout do Sensor de Movimento --- */
#define MOTION_SENSOR 35

/* --- Pinout do Buzzer --- */
#define BUZZER 32

/* --- LEDs --- */
#define LED_BLUE 27
#define LED_RED 12

/* --- Variáveis --- */
volatile bool alarme = false;    // FLAG do alarme
volatile bool state = false;     // FLAG de movimento detetado
volatile bool timeOut = false;  // FLAG de Time Out - após 10s à espera de uma operação em certos ecrãs
volatile uint8_t failCount = 0; // Contador de tentativas falhadas ao introduzir pass (max 3)

/* --- Timer: Auxiliary variables --- */
unsigned long now = millis();
unsigned long nowBuzzer = millis();
bool toggleBuzzer = false;
bool buzzerON = false;
unsigned long lastTrigger = 0;
unsigned long lastBuzzer = 0;
bool startTimer = false;


/* --- PWM   --- */
const int freq = 5000;
const int buzzerChannel = 0;
const int resolution = 8;

/* --- FUNÇÕES NAVEGAÇÃO --- */
uint8_t menu_select(bool main); // bool main -> Flag que indica se o user se encontra no menu_main ou não
uint8_t menu_MoveUp(uint8_t yCursor);
uint8_t menu_MoveDown(uint8_t yCursor);
uint8_t submenu_select(uint8_t yCursor);
uint8_t submenu_select_2();
uint8_t op;

/* --- MENUS --- */
uint8_t menu_main();    // Menu Main: 1 - alarme, 2 - Settings, 3 - Debug
uint8_t menu_alarm();   // Menu alarme: Arm/Disarm alarme
uint8_t menu_settings();// Menu ?
void menu_debug();      // Debug

/* --- alarme --- */
void alarme_disarm();    // Desarma alarmee
void alarme_arm();       // Arma alarmee
void read_key();        // Pede leitura de chave

/* --- STATIC SCREENS --- */
void permission_denied();
void time_out();        // Ecrã "TIME OUT" após tempo limite de resposta ultrapassado
void key();             // Ecrã "INSERT KEY" que sinaliza a leitura de inserção de chave
void alarme_on();        // Ecrã "ALARM ON"
void alarme_off();       // Ecrã "ALARM OFF"

/* --- Outras Funções --- */
void serial_flush();    // Limpa buffer do input do terminal


/* --- Checks if motion was detected, sets LED HIGH and starts a timer --- */
void IRAM_ATTR detectsMovement() {
  state = 1;
}

/*--------------------------------------------------------------------------------------\
  |                                       MAIN                                            |
  \--------------------------------------------------------------------------------------*/
void setup() {
  Serial.begin(115200);
  tft.begin();          // Inicializa o LCD
  tft.setRotation(1);   // Landscape mode
  tft.setTextSize(3);   // Tamanho texto
  tft.setTextWrap(true);// \n automático após chegar ao limite do ecrã
  pinMode(X, INPUT);          // Joystick X
  pinMode(Y, INPUT);          // Joystick Y
  pinMode(SW, INPUT_PULLUP);  // Joystick Button
  pinMode(BUZZER, OUTPUT);    // Buzzer
  pinMode(MOTION_SENSOR, INPUT_PULLUP);// Sensor de Movimento
  pinMode(LED_RED, OUTPUT);     // 12
  pinMode(LED_BLUE, OUTPUT);   // 27
  digitalWrite(LED_RED, LOW);   // 12
  digitalWrite(LED_BLUE, HIGH);// 27
  attachInterrupt(digitalPinToInterrupt(MOTION_SENSOR), detectsMovement, RISING);

  ledcSetup(buzzerChannel, freq, resolution);
  ledcAttachPin(BUZZER, buzzerChannel);
}

/*-------------------------------------------------------------------------------------\
  |                                       LOOP                                           |
  \-------------------------------------------------------------------------------------*/
void loop() {
  Serial.println("0 - Menu Main");
  op = menu_main();
  switch (op) {
    case 1: // alarme
      Serial.println("1 - Alarm\n");
      if (!alarme) {
        op = menu_alarm();
        if (op == 1) {
          alarme_arm();
        }
      } else {
        alarme_disarm();
      }
      break;
    case 2: // SETTINGS
      Serial.println("2 - Settings\n");
      break;
    case 3: // DEBUG
      Serial.println("3 - Debug\n");
      menu_debug();
      break;
    default:
      Serial.println("ERROR: Invalid Option\n");
  }
}

/*-------------------------------------------------------------------------------------\
  |                                     FUNÇÕES                                          |
  \-------------------------------------------------------------------------------------*/
/*-------------------------------------------------------------------------------------\
  |                                 ALARME / ALARM                                       |
  \-------------------------------------------------------------------------------------*/

void motionSensorOn() {
  if (state) {
    Serial.println(" MOTION DETECTED! ");
    if (!buzzerON) {
      lastBuzzer = millis();
    }
    buzzerON = true;
    state = false;
  }
  
  if (buzzerON) {
    digitalWrite(LED_BLUE, LOW);
    nowBuzzer = millis();
    if (nowBuzzer - lastBuzzer > 250) {
      if (toggleBuzzer) {
        ledcWrite(buzzerChannel, 255);
        toggleBuzzer = false;
        digitalWrite(LED_RED, HIGH);
      } else {
        ledcWrite(buzzerChannel, 127);
        toggleBuzzer = true;
        digitalWrite(LED_RED, LOW);
      }
      lastBuzzer = millis();
    }
  }
}

//---------- ALARM ARM ----------
void alarme_arm() {
  alarme_on();   // Red Screen
  alarme = true; // FLAG ALARM ON
}

//---------- ALARM DISARM ----------
void alarme_disarm() {
  alarme = false;// FLAG ALARM OFF
  digitalWrite(LED_RED, LOW);
  digitalWrite(LED_BLUE, HIGH);
  ledcWrite(buzzerChannel, 0);
  toggleBuzzer = false;
  buzzerON = false;
  alarme_off();  // GREEN SCREEN
}

//---------- READ KEY ----------
void read_key() {
  key();    // "INSERT KEY" Screen
  unsigned long previousMillis = 0, currentMillis;  // millis();
  bool estado = false;   // Usado para millis();

  do {
    currentMillis  = millis();
    if (!estado) {
      previousMillis = millis();
      estado = true;
    }
    if (currentMillis - previousMillis >= 10000) {
      time_out();    // Timeout
      return;
    }
  } while (true);
}

/*-------------------------------------------------------------------------------------\
  |                                     MENUS                                            |
  \-------------------------------------------------------------------------------------*/
//---------- MAIN MENU ----------
uint8_t menu_main() {
  uint8_t op;

  if (!alarme) {
    tft.fillScreen(ILI9341_YELLOW); // Background color
    // 1ª opção
    tft.fillRoundRect(25, 22.5, 270, 50, 25, ILI9341_BLACK);  //tft.fillRect(25,22.5,270,50, ILI9341_BLACK);  // x_inicio, y_inicio, x_size, y_size, color
    tft.setCursor(115, 35);    // x, y
    tft.println("ALARM");
    // 2ª opção
    tft.fillRoundRect(25, 95, 270, 50, 25, ILI9341_BLACK); //tft.fillRect(25,95,270,50, ILI9341_BLACK);    // x_inicio, y_inicio, x_size, y_size, color
    tft.setCursor(90, 107.5);  // x, y
    tft.println("Settings");
    // 3ª opção
    tft.fillRoundRect(25, 167.5, 270, 50, 25, ILI9341_BLACK);  //tft.fillRect(25,167.5,270,50, ILI9341_BLACK); // x_inicio, y_inicio, x_size, y_size, color
    tft.setCursor(105, 180);   // x, y
    tft.println("Cenas");

    Serial.print("Selected Menu: ");
    op = menu_select(true);

  } else {
    tft.fillScreen(ILI9341_RED); // Background color
    tft.fillRoundRect(25, 95, 270, 50, 25, ILI9341_BLACK); //tft.fillRect(25,95,270,50, ILI9341_BLACK);    // x_inicio, y_inicio, x_size, y_size, color
    tft.setCursor(90, 107.5);  // x, y
    tft.println("DISARM");
    op = submenu_select(1);
  }
  serial_flush();
  return op;  // 1 = Alarm, 2 = Settings, 3 = Cenas
}

//---------- MENU ALARM (1) ----------
uint8_t menu_alarm() {
  uint8_t op;

  tft.fillScreen(ILI9341_YELLOW); // Background color
  tft.fillRoundRect(25, 95, 270, 50, 25, ILI9341_BLACK);    // x_inicio, y_inicio, x_size, y_size, color
  tft.setCursor(69, 107.5);  // x, y
  tft.println("Arm");
  op = submenu_select(2);
  return op;
}

//---------- MENU SETTINGS (2) ----------
uint8_t menu_settings() {
  uint8_t op;

  tft.fillScreen(ILI9341_YELLOW); // Background color
  // 1ª opção
  tft.fillRoundRect(25, 22.5, 270, 50, 25, ILI9341_BLACK);  //tft.fillRect(25,22.5,270,50, ILI9341_BLACK);  // x_inicio, y_inicio, x_size, y_size, color
  tft.setCursor(100, 35);    // x, y
  tft.println("");
  // 2ª opção
  tft.fillRoundRect(25, 95, 270, 50, 25, ILI9341_BLACK);    //tft.fillRect(25,95,270,50, ILI9341_BLACK);    // x_inicio, y_inicio, x_size, y_size, color
  tft.setCursor(100, 107.5);  // x, y
  tft.println("");
  // 3ª opção
  tft.fillRoundRect(25, 167.5, 270, 50, 25, ILI9341_BLACK); //tft.fillRect(25,167.5,270,50, ILI9341_BLACK); // x_inicio, y_inicio, x_size, y_size, color
  tft.setCursor(100, 180);   // x, y
  tft.println("");

  op = menu_select(false);

  return op;
}

//---------- MENU DEBUG (3) ----------
void menu_debug() { // Menu usado para debugging (temporário)

}

/*-------------------------------------------------------------------------------------\
  |                              NAVEGAÇÃO / NAVEGATION                                  |
  \-------------------------------------------------------------------------------------*/
//---------- Função de navegação em menus de 3 opções ----------
uint8_t menu_select(bool main) {
  uint8_t op = 1, yCursor = 47;
  tft.fillCircle(45, yCursor, 15, ILI9341_RED);
  do {
    if (analogRead(Y) < 500) { // Para cima
      switch (op) {
        case 1:
          op = 3;
          break;
        default:
          op--;
      }
      yCursor = menu_MoveUp(yCursor);
      while (analogRead(Y) < 500) { }
    }

    if (analogRead(Y) > 3500) { // Para baixo
      switch (op) {
        case 3:
          op = 1;
          break;
        default:
          op++;
      }
      yCursor = menu_MoveDown(yCursor);
      while (analogRead(Y) > 3500) { }
    }

    if (analogRead(X) < 500 && main == false) { // Para trás
      return 0; // Back
    }
  } while (analogRead(SW) > 500 && analogRead(X) < 3500);
  return op;
}

//---------- Joystick Move Up ----------
uint8_t menu_MoveUp(uint8_t yCursor) {
  if (yCursor == 47) {
    tft.fillCircle(45, yCursor, 15, ILI9341_BLACK);
    yCursor = 191;
    tft.fillCircle(45, yCursor, 15, ILI9341_RED);
  } else {
    tft.fillCircle(45, yCursor, 15, ILI9341_BLACK);
    yCursor = yCursor - 72;
    tft.fillCircle(45, yCursor, 15, ILI9341_RED);
  }
  return yCursor;
}

//---------- Joystick Move Down ----------
uint8_t menu_MoveDown(uint8_t yCursor) {
  if (yCursor == 191) {
    tft.fillCircle(45, yCursor, 15, ILI9341_BLACK);
    yCursor = 47;
    tft.fillCircle(45, yCursor, 15, ILI9341_RED);
  } else {
    tft.fillCircle(45, yCursor, 15, ILI9341_BLACK);
    yCursor = yCursor + 72;
    tft.fillCircle(45, yCursor, 15, ILI9341_RED);
  }
  return yCursor;
}

//---------- Função de navegação em menus de 1 opção ----------
uint8_t submenu_select(uint8_t option) { // option = onde esta se encontra no ecrã
  uint8_t yCursor;
  switch (option) {
    case 1:
      yCursor = 47;
      break;
    case 3:
      yCursor = 191;
      break;
    default:
      yCursor = 119;
  }
  tft.fillCircle(45, yCursor, 15, ILI9341_RED);

  do {
    if (alarme) motionSensorOn();
    if (analogRead(X) < 500) return 0;  // Para trás
  } while (analogRead(SW) > 500 && analogRead(X) < 3500);
  return 1;
}





//---------- Função de navegação em menus de 2 opção ----------
uint8_t submenu_select_2() {    // Para menus com 2 opções (1ª e 2ª)
  uint8_t op = 1;
  tft.fillCircle(45, 47, 15, ILI9341_RED);
  do {
    if (analogRead(Y) < 500 || analogRead(Y) > 3500) {
      switch (op) {
        case 1:
          op = 2;
          tft.fillCircle(45, 119, 15, ILI9341_RED);
          tft.fillCircle(45, 47, 15, ILI9341_BLACK);
          break;
        case 2:
          op = 1;
          tft.fillCircle(45, 47, 15, ILI9341_RED);
          tft.fillCircle(45, 119, 15, ILI9341_BLACK);
          break;
      } while (analogRead(Y) < 500 || analogRead(Y) > 3500) { }
    }

    if (analogRead(X) < 500) return 0; // Para trás
  } while (analogRead(SW) > 500 && analogRead(X) < 3500);
  return op;
}

/*-------------------------------------------------------------------------------------\
  |                        ECRÃS ESTÁTICOS / STATIC SCREENS                              |
  \-------------------------------------------------------------------------------------*/
//---------- PERMISSION DENIED SCREEN ----------
void permission_denied() {
  tft.fillScreen(ILI9341_BLACK);  // Background color
  tft.setCursor(30, 107.5);
  tft.println("ACCESS DENIED");
  delay(1000);  // Show for ~1.5s, "smart delay"
}

//---------- INSERT KEY SCREEN ----------
void key() {
  tft.fillScreen(ILI9341_BLACK);
  tft.setCursor(75, 108);
  tft.println("PASSWORD");  // huawei
}

//---------- TIME OUT SCREEN ----------
void time_out() {
  tft.fillScreen(ILI9341_BLACK);  // Background color
  tft.setCursor(60, 107.5);
  tft.println("TIMED OUT");
}

//---------- ALARM ARMED SCREEN ----------
void alarme_on() {
  tft.fillScreen(ILI9341_RED);    // Background color
  tft.setCursor(90, 100);
  tft.println("ALARM ON");
  for (uint8_t i = 10 ; i > 0 ; i--) {
    if (i == 10) {
      tft.setCursor(136, 170);
    } else {
      tft.setCursor(142, 170);
    }
    tft.fillCircle(150, 180, 30, ILI9341_BLACK);
    tft.print(i);
    delay(1000);    // Show for 10s (counter), "smart delay"
  }
}

//---------- ALARM DISARMED SCREEN ----------
void alarme_off() {
  tft.fillScreen(ILI9341_GREEN);  // Background color
  tft.setCursor(90, 100);
  tft.println("ALARM OFF");
  delay(1000);  // Show for ~2s, "smart delay"
}

/*-------------------------------------------------------------------------------------\
  |                                 OUTROS / OTHERS                                      |
  \-------------------------------------------------------------------------------------*/
//---------- SERIAL FLUSH ----------
void serial_flush() {
  while (Serial.available()) Serial.read();
}
