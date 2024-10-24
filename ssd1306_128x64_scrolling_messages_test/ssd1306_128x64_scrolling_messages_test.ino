#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// Definiere die Displaygröße
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

// Definiere den Reset-Pin (falls verwendet)
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Maximale Anzahl der alten Nachrichten, die auf dem Display angezeigt werden können
#define MAX_OLD_MESSAGES ((SCREEN_HEIGHT - 16) / 8) // Die unterste Zeile ist für die neueste Nachricht reserviert

// Maximale Länge einer Nachricht in Zeichen
#define MAX_MESSAGE_LENGTH (SCREEN_WIDTH / 6) // Da jeder Charakter etwa 6 Pixel breit ist

// Array zur Speicherung der alten Nachrichten
String messages[MAX_OLD_MESSAGES];
int messageCount = 0;

// Initialisierungsfunktion
void setup() {
  // Initialisiere das Display
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Überprüfe die I2C Adresse deines Displays
    Serial.println(F("SSD1306 allocation failed"));
    for (;;);
  }
  display.clearDisplay();
  display.display();
}

// Funktion zum Hinzufügen einer neuen Nachricht
void addMessage(String newMessage) {
  // Kürze die Nachricht, wenn sie zu lang ist
  if (newMessage.length() > MAX_MESSAGE_LENGTH) {
    newMessage = newMessage.substring(0, MAX_MESSAGE_LENGTH);
  }

  // Wenn die maximale Anzahl der alten Nachrichten erreicht ist, verschiebe die Nachrichten nach oben
  if (messageCount >= MAX_OLD_MESSAGES) {
    for (int i = 0; i < MAX_OLD_MESSAGES - 1; i++) {
      messages[i] = messages[i + 1];
    }
    messages[MAX_OLD_MESSAGES - 1] = newMessage;
  } else {
    messages[messageCount] = newMessage;
    messageCount++;
  }

  // Zeige die Nachrichten auf dem Display an
  displayMessages();
}

// Funktion zur Anzeige der Nachrichten auf dem Display
void displayMessages() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  // Zeichne die alten Nachrichten bis zur Linie
  for (int i = 0; i < messageCount - 1; i++) {
    display.setCursor(0, i * 8); // Setze den Cursor auf die entsprechende Zeile
    display.println(messages[i]);
    if ((i + 1) * 8 >= SCREEN_HEIGHT - 16) break; // Stoppe, wenn die Linie erreicht ist
  }

  // Zeichne die Trennlinie
  display.drawLine(0, SCREEN_HEIGHT - 17, SCREEN_WIDTH, SCREEN_HEIGHT - 17, SSD1306_WHITE);

  // Zeichne die neue Nachricht nur unter der Linie
  if (messageCount > 0) {
    display.setCursor(0, SCREEN_HEIGHT - 8);
    display.println(messages[messageCount - 1]);
  }

  display.display();
}

void loop() {
  // Beispielaufrufe zum Testen
  addMessage("Nachricht 1");
  delay(1000);
  addMessage("Nachricht 2");
  delay(1000);
  addMessage("Nachricht 3");
  delay(1000);
  addMessage("Nachricht 4");
  delay(1000);
  addMessage("Nachricht 5");
  delay(1000);
  addMessage("Nachricht 6");
  delay(1000);
}
