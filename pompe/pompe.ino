#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>

// WiFi credentials
const char* ssid = "FUN5";
const char* password = "atout.be";

// Déclaration des broches relais
const int RELAIS_ONOFF = D4;
const int RELAIS_V1    = D1;
const int RELAIS_V2    = D2;
const int RELAIS_V3    = D3;

// Détecteur de débit via optocoupleur
const int FLOW_PIN     = D5;

// Serveur Web
ESP8266WebServer server(80);

int vitesseActuelle = 0; // 0 = OFF

// ----------------------
// DETECTION DU DEBIT
// ----------------------
bool flowDetecte() {
  // Avec notre montage :
  // HIGH = flow switch fermé = débit présent
  // LOW  = flow switch ouvert = pas de débit
  return digitalRead(FLOW_PIN) == HIGH;
}

// ----------------------
// ARRET COMPLET
// ----------------------
void pompeArret() {
  Serial.println("Commande: pompeArret");

  digitalWrite(RELAIS_V1, HIGH);
  digitalWrite(RELAIS_V2, HIGH);
  digitalWrite(RELAIS_V3, HIGH);

  delay(200); // sécurité

  digitalWrite(RELAIS_ONOFF, HIGH); // OFF

  vitesseActuelle = 0;
}

// ----------------------
// CHANGEMENT VITESSE
// ----------------------
void pompeVitesse(int vitesse) {
  Serial.print("Commande: pompeVitesse(");
  Serial.print(vitesse);
  Serial.println(")");

  // Si pompe OFF → démarrage
  if (vitesseActuelle == 0) {
    digitalWrite(RELAIS_ONOFF, LOW); // ON
    delay(300);
  }

  // On coupe uniquement les vitesses
  digitalWrite(RELAIS_V1, HIGH);
  digitalWrite(RELAIS_V2, HIGH);
  digitalWrite(RELAIS_V3, HIGH);

  delay(300); // temps de sécurité

  // Activation vitesse
  switch(vitesse) {
    case 1:
      digitalWrite(RELAIS_V1, LOW);
      break;

    case 2:
      digitalWrite(RELAIS_V2, LOW);
      break;

    case 3:
      digitalWrite(RELAIS_V3, LOW);
      break;
  }

  vitesseActuelle = vitesse;
}

// ----------------------
// API
// ----------------------
void handleRoot() {
  server.send(200, "text/plain", "Pompe Hayward API OK");
}

void handlePompeOn() {
  pompeVitesse(1);
  server.send(200, "text/plain", "Pompe ON (Vitesse 1)");
}

void handlePompeOff() {
  pompeArret();
  server.send(200, "text/plain", "Pompe OFF");
}

void handlePompeVitesse() {
  if (server.hasArg("v")) {
    int v = server.arg("v").toInt();

    if (v >= 1 && v <= 3) {
      pompeVitesse(v);

      server.send(
        200,
        "text/plain",
        String("Pompe ON (Vitesse ") + v + ")"
      );

      return;
    }
  }

  server.send(
    400,
    "text/plain",
    "Parametre ?v=1|2|3 requis"
  );
}

// ----------------------
// STATUS COMPLET
// ----------------------
void handlePompeStatus() {

  bool flow = flowDetecte();

  String statut = "{";

  if (vitesseActuelle == 0) {

    statut += "\"etat\":\"off\"";

  } else {

    statut += "\"etat\":\"on\",";
    statut += "\"vitesse\":";
    statut += vitesseActuelle;
  }

  statut += ",\"flow\":";
  statut += flow ? "true" : "false";

  statut += "}";

  server.send(
    200,
    "application/json",
    statut
  );
}

// ----------------------
// STATUS FLOW UNIQUEMENT
// ----------------------
void handleFlowStatus() {

  bool flow = flowDetecte();

  String statut = "{";
  statut += "\"flow\":";
  statut += flow ? "true" : "false";
  statut += "}";

  server.send(
    200,
    "application/json",
    statut
  );
}

// ----------------------
// SETUP
// ----------------------
void setup() {

  Serial.begin(115200);

  Serial.println();
  Serial.println("------ DEMARRAGE POMPE ------");

  // Relais
  pinMode(RELAIS_ONOFF, OUTPUT);
  pinMode(RELAIS_V1, OUTPUT);
  pinMode(RELAIS_V2, OUTPUT);
  pinMode(RELAIS_V3, OUTPUT);

  // Détecteur de débit
  pinMode(FLOW_PIN, INPUT);

  // Sécurité démarrage
  digitalWrite(RELAIS_ONOFF, HIGH);
  digitalWrite(RELAIS_V1, HIGH);
  digitalWrite(RELAIS_V2, HIGH);
  digitalWrite(RELAIS_V3, HIGH);

  delay(500);

  // WiFi
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  Serial.print("Connexion WiFi");

  int compteur = 0;

  while (WiFi.status() != WL_CONNECTED) {

    delay(500);
    Serial.print(".");

    compteur++;

    if (compteur > 30) {
      Serial.println("\nEchec WiFi -> reboot");
      ESP.restart();
    }
  }

  Serial.println("\nWiFi connecte !");

  Serial.print("IP: ");
  Serial.println(WiFi.localIP());

  // Routes
  server.on("/", handleRoot);

  server.on("/pompe/on", handlePompeOn);
  server.on("/pompe/off", handlePompeOff);
  server.on("/pompe/vitesse", handlePompeVitesse);

  server.on("/pompe/status", handlePompeStatus);

  // Nouvelle route débit
  server.on("/pompe/flow", handleFlowStatus);

  server.begin();

  Serial.println("Serveur HTTP demarre !");
}

// ----------------------
// LOOP
// ----------------------
void loop() {

  server.handleClient();

}
