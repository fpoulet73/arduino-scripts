#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <EEPROM.h>

// Pompe Hayward + lecture Flow local + diagnostic AstralPool Modbus RTU.
// ESP8266 UART0: TX(GPIO1) -> RXD module, RX(GPIO3) <- TXD module (verifier convention du module).
// UART0 est partage avec l'USB : debrancher le module si le flash echoue.
// Ne connecter que sur une interface TTL 3.3V certifiee.
// RS485 A/B vers AstralPool A/B; liaison de reference seulement si documentation validee.
// Ne PAS relier le bus RS485 au 230V ni aux electrodes pH/ORP.

const char* WIFI_SSID = "FUN5";
const char* WIFI_PASS = "atout.be";
const uint8_t RELAIS_ONOFF=D4, RELAIS_V1=D1, RELAIS_V2=D2, RELAIS_V3=D3, FLOW_PIN=D5;
ESP8266WebServer server(80);
int vitesseActuelle=0;

// Les valeurs Modbus ci-dessous sont des PARAMETRES DE TEST, pas des valeurs confirmees.
struct Config { uint32_t magic; uint32_t baud; uint8_t slave, parity; uint16_t timeoutMs; } cfg;
const uint32_t MAGIC=0xA5744B11;
uint32_t okCount=0, timeoutCount=0, crcCount=0, exceptionCount=0, framingCount=0, txCount=0;
String lastResult="Aucun test", lastHex="", lastError="";
uint32_t lastResponseMs=0;
String lastTxHex="", lastRxHex="";
uint32_t lastRxBytes=0, lastFirstByteMs=0, lastInterByteMs=0, lastTxFlushMs=0;
bool lastOverflow=false;

String jsonEscape(const String &s) {
  String r; r.reserve(s.length()+8);
  for (size_t i=0;i<s.length();i++) {
    char c=s[i];
    if(c=='"'||c=='\\'){r+='\\';r+=c;}
    else if(c=='\n')r+="\\n";
    else if(c=='\r')r+="\\r";
    else if((uint8_t)c>=32)r+=c;
  }
  return r;
}
String hexBytes(const uint8_t* data, size_t n) {
  const char* h="0123456789ABCDEF"; String s; s.reserve(n*3);
  for(size_t i=0;i<n;i++){if(i)s+=' ';s+=h[data[i]>>4];s+=h[data[i]&15];} return s;
}
uint16_t crc16(const uint8_t *data, size_t n) {
  uint16_t crc=0xFFFF;
  for(size_t i=0;i<n;i++){crc^=data[i];for(int b=0;b<8;b++)crc=(crc&1)?((crc>>1)^0xA001):(crc>>1);}
  return crc;
}
void saveConfig(){EEPROM.put(0,cfg);EEPROM.commit();}
void beginModbus(){
  Serial.flush(); Serial.end();
  SerialConfig sc=SERIAL_8N1;
  if(cfg.parity==1)sc=SERIAL_8E1;
  if(cfg.parity==2)sc=SERIAL_8O1;
  Serial.begin(cfg.baud,sc);
  while(Serial.available())Serial.read();
}
void initConfig(){
  EEPROM.begin(64);EEPROM.get(0,cfg);
  if(cfg.magic!=MAGIC || cfg.baud<1200 || cfg.baud>115200 || cfg.slave<1 || cfg.slave>247 || cfg.parity>2 || cfg.timeoutMs<100 || cfg.timeoutMs>3000){
    cfg.magic=MAGIC;cfg.baud=19200;cfg.slave=1;cfg.parity=0;cfg.timeoutMs=600;saveConfig();
  }
}

bool flowDetecte(){return digitalRead(FLOW_PIN)==HIGH;}
void pompeArret(){
  digitalWrite(RELAIS_V1,HIGH);digitalWrite(RELAIS_V2,HIGH);digitalWrite(RELAIS_V3,HIGH);
  delay(200);digitalWrite(RELAIS_ONOFF,HIGH);vitesseActuelle=0;
}
void pompeVitesse(int vitesse){
  if(vitesse<1||vitesse>3)return;
  if(vitesseActuelle==0){digitalWrite(RELAIS_ONOFF,LOW);delay(300);}
  digitalWrite(RELAIS_V1,HIGH);digitalWrite(RELAIS_V2,HIGH);digitalWrite(RELAIS_V3,HIGH);
  delay(300);
  if(vitesse==1)digitalWrite(RELAIS_V1,LOW);
  if(vitesse==2)digitalWrite(RELAIS_V2,LOW);
  if(vitesse==3)digitalWrite(RELAIS_V3,LOW);
  vitesseActuelle=vitesse;
}

struct Reply { bool ok=false; uint8_t exception=0; uint16_t values[32]; uint8_t count=0; String error="", raw=""; uint32_t ms=0; };
// Lecture strictement limitee aux fonctions 03/04, maximum 32 registres.
Reply modbusRead(uint8_t slave,uint8_t fn,uint16_t address,uint8_t count){
  Reply result;
  if(slave<1||slave>247|| (fn!=3&&fn!=4)||count<1||count>32 || (uint32_t)address+count>65536){result.error="Parametres invalides";return result;}
  uint8_t req[8]={slave,fn,(uint8_t)(address>>8),(uint8_t)address,0,count,0,0};
  uint16_t crc=crc16(req,6);req[6]=crc&255;req[7]=crc>>8;
  lastTxHex=hexBytes(req,8);
  while(Serial.available())Serial.read();
  // Certains convertisseurs auto-direction necessitent un temps de bascule.
  delay(3);uint32_t start=millis();Serial.write(req,8);Serial.flush();lastTxFlushMs=millis()-start;txCount++;
  uint8_t rx[80];size_t n=0;uint32_t lastByte=start;uint32_t firstByte=0;bool overflow=false;
  while(millis()-start<cfg.timeoutMs){
    while(Serial.available()){
      int b=Serial.read();
      if(!n)firstByte=millis();
      if(n<sizeof(rx))rx[n++]=(uint8_t)b;else overflow=true;
      lastByte=millis();
    }
    if(n>=5 && millis()-lastByte>=12)break;
    yield();
  }
  result.ms=millis()-start;lastResponseMs=result.ms;result.raw=hexBytes(rx,n);lastHex=result.raw;
  lastRxHex=result.raw;lastRxBytes=n;lastFirstByteMs=n?firstByte-start:0;lastInterByteMs=n?lastByte-firstByte:0;lastOverflow=overflow;
  if(overflow){result.error="Tampon reception sature (>80 octets)";framingCount++;}
  else if(n==0){result.error="Timeout: aucune reponse";timeoutCount++;}
  else if(n<5){result.error="Trame incomplete";framingCount++;}
  else {
    // Tolere eventuel echo de la requete par le convertisseur.
    size_t offset=0;
    if(n>=13 && memcmp(rx,req,8)==0)offset=8;
    size_t len=n-offset;
    if(len<5){result.error="Reponse trop courte";framingCount++;}
    else {
      uint16_t received=rx[n-2]|((uint16_t)rx[n-1]<<8);
      if(crc16(rx+offset,len-2)!=received){result.error="CRC invalide";crcCount++;}
      else if(rx[offset]!=slave){result.error="Adresse esclave inattendue";framingCount++;}
      else if(rx[offset+1]==(fn|0x80)){
        result.exception=rx[offset+2];result.error="Exception Modbus "+String(result.exception);exceptionCount++;
      }
      else if(rx[offset+1]!=fn || rx[offset+2]!=count*2 || len!=(size_t)(5+count*2)){
        result.error="Fonction/taille de reponse inattendue";framingCount++;
      } else {
        result.ok=true;result.count=count;okCount++;
        for(uint8_t i=0;i<count;i++)result.values[i]=((uint16_t)rx[offset+3+2*i]<<8)|rx[offset+4+2*i];
      }
    }
  }
  lastResult=result.ok?"OK":result.error;lastError=result.ok?"":result.error;
  return result;
}

bool busy=false;
String replyJson(const Reply& r,uint8_t slave,uint8_t fn,uint16_t addr){
  String s="{\"ok\":";s+=(r.ok?"true":"false");
  s+=",\"slave\":"+String(slave)+",\"function\":"+String(fn)+",\"address\":"+String(addr);
  s+=",\"duration_ms\":"+String(r.ms)+",\"error\":\""+jsonEscape(r.error)+"\",\"exception\":"+String(r.exception);
  s+=",\"raw_hex\":\""+jsonEscape(r.raw)+"\",\"tx_hex\":\""+lastTxHex+"\",\"rx_bytes\":"+String(lastRxBytes);
  s+=",\"first_byte_ms\":"+String(lastFirstByteMs)+",\"rx_span_ms\":"+String(lastInterByteMs)+",\"tx_flush_ms\":"+String(lastTxFlushMs);
  s+=",\"rx_overflow\":"+String(lastOverflow?"true":"false")+",\"registers\":[";
  for(uint8_t i=0;i<r.count;i++){if(i)s+=",";s+=String(r.values[i]);}s+="]}";return s;
}
void sendJson(const String &s,int status=200){server.sendHeader("Cache-Control","no-store");server.send(status,"application/json; charset=utf-8",s);}
void handleRoot(){
  String html=F("<!doctype html><html lang='fr'><meta name='viewport' content='width=device-width,initial-scale=1'><meta charset='utf-8'><title>Pompe / AstralPool</title><style>body{font:16px sans-serif;max-width:820px;margin:24px auto;padding:0 12px}input{width:85px;padding:6px}button{padding:8px;margin:4px}pre{white-space:pre-wrap;overflow-wrap:anywhere;background:#eee;padding:12px}label{display:inline-block;margin:6px}</style><h2>Pompe et diagnostic AstralPool</h2><p>Lecture Modbus seulement (FC03/FC04). Aucun registre n'est ecrit.</p><p><a href='/pompe/status'>Pompe / Flow</a> | <a href='/astral/status'>Etat RS485</a></p><h3>Configuration UART</h3><label>Baud <input id='baud' type='number'></label><label>Esclave <input id='slave' type='number'></label><label>Parite (0=N,1=E,2=O) <input id='parity' type='number'></label><label>Timeout ms <input id='timeout' type='number'></label><button onclick='config()'>Enregistrer</button><h3>Lecture Modbus</h3><label>Fonction <input id='fn' type='number' value='3'></label><label>Adresse (0-based) <input id='addr' type='number' value='0'></label><label>Nombre (1-32) <input id='count' type='number' value='1'></label><button onclick='read()'>Lire</button><h3>Recherche manuelle limitee</h3><p>Balayage maximum 64 registres par appel, en lecture seule. Eviter les scans repetes.</p><label>Debut <input id='start' type='number' value='0'></label><label>Nombre <input id='total' type='number' value='16'></label><button onclick='scan()'>Scanner</button><pre id='out'>Chargement...</pre><script>const g=x=>document.getElementById(x).value;async function api(p){try{let r=await fetch(p);let t=await r.text();try{document.getElementById('out').textContent=JSON.stringify(JSON.parse(t),null,2)}catch(e){document.getElementById('out').textContent=t}}catch(e){document.getElementById('out').textContent=String(e)}}async function load(){let d=await(await fetch('/astral/status')).json();['baud','slave','parity','timeout'].forEach(k=>document.getElementById(k).value=d[k]);document.getElementById('out').textContent=JSON.stringify(d,null,2)}function config(){api('/astral/config?baud='+g('baud')+'&slave='+g('slave')+'&parity='+g('parity')+'&timeout='+g('timeout'))}function read(){api('/astral/read?fn='+g('fn')+'&addr='+g('addr')+'&count='+g('count'))}function scan(){api('/astral/scan?fn='+g('fn')+'&start='+g('start')+'&count='+g('total'))}load()</script></html>");
  server.send(200,"text/html; charset=utf-8",html);
}
void handlePompeStatus(){String s="{\"etat\":\""+String(vitesseActuelle?"on":"off")+"\",\"vitesse\":"+String(vitesseActuelle)+",\"flow\":"+String(flowDetecte()?"true":"false")+"}";sendJson(s);}
void handleFlowStatus(){sendJson(String("{\"flow\":")+(flowDetecte()?"true":"false")+"}");}
void handleAstralStatus(){
  String s="{\"baud\":"+String(cfg.baud)+",\"slave\":"+String(cfg.slave)+",\"parity\":"+String(cfg.parity)+",\"timeout\":"+String(cfg.timeoutMs);
  s+=",\"busy\":"+String(busy?"true":"false")+",\"requests\":"+String(txCount)+",\"ok\":"+String(okCount)+",\"timeouts\":"+String(timeoutCount);
  s+=",\"crc_errors\":"+String(crcCount)+",\"exceptions\":"+String(exceptionCount)+",\"frame_errors\":"+String(framingCount);
  s+=",\"last_duration_ms\":"+String(lastResponseMs)+",\"last_result\":\""+jsonEscape(lastResult)+"\",\"last_hex\":\""+jsonEscape(lastHex)+"\"";
  s+=",\"last_tx_hex\":\""+lastTxHex+"\",\"last_rx_bytes\":"+String(lastRxBytes)+",\"last_first_byte_ms\":"+String(lastFirstByteMs);
  s+=",\"last_rx_span_ms\":"+String(lastInterByteMs)+",\"last_tx_flush_ms\":"+String(lastTxFlushMs)+",\"last_rx_overflow\":"+String(lastOverflow?"true":"false")+"}";sendJson(s);
}
bool parseUint(const String& key,uint32_t& value,uint32_t lo,uint32_t hi){
  if(!server.hasArg(key))return false;
  String s=server.arg(key);if(s.length()==0)return false;
  for(size_t i=0;i<s.length();i++)if(s[i]<'0'||s[i]>'9')return false;
  unsigned long n=strtoul(s.c_str(),nullptr,10);if(n<lo||n>hi)return false;value=n;return true;
}
void handleConfig(){
  if(busy){sendJson("{\"error\":\"busy\"}",409);return;}
  uint32_t baud,slave,parity,timeout;
  if(!parseUint("baud",baud,1200,115200)||!parseUint("slave",slave,1,247)||!parseUint("parity",parity,0,2)||!parseUint("timeout",timeout,100,3000)){
    sendJson("{\"error\":\"baud/slave/parity/timeout invalides\"}",400);return;
  }
  cfg.baud=baud;cfg.slave=slave;cfg.parity=parity;cfg.timeoutMs=timeout;saveConfig();beginModbus();handleAstralStatus();
}
void handleRead(){
  if(busy){sendJson("{\"error\":\"busy\"}",409);return;}
  uint32_t fn,addr,count,slave=cfg.slave;
  if(!parseUint("fn",fn,3,4)||!parseUint("addr",addr,0,65535)||!parseUint("count",count,1,32)||addr+count>65536 || (server.hasArg("slave")&&!parseUint("slave",slave,1,247))){
    sendJson("{\"error\":\"Parametres invalides\"}",400);return;
  }
  busy=true;Reply r=modbusRead(slave,fn,addr,count);busy=false;sendJson(replyJson(r,slave,fn,addr));
}
void handleScan(){
  if(busy){sendJson("{\"error\":\"busy\"}",409);return;}
  uint32_t fn,start,count;
  if(!parseUint("fn",fn,3,4)||!parseUint("start",start,0,65535)||!parseUint("count",count,1,64)||start+count>65536){sendJson("{\"error\":\"Parametres invalides\"}",400);return;}
  busy=true;
  String s="{\"start\":"+String(start)+",\"count\":"+String(count)+",\"function\":"+String(fn)+",\"blocks\":[";
  for(uint32_t i=0;i<count;i+=8){
    if(i)s+=",";
    uint8_t n=(count-i>=8)?8:(count-i);
    Reply r=modbusRead(cfg.slave,fn,start+i,n);
    s+=replyJson(r,cfg.slave,fn,start+i);
    yield();
  }
  s+="]}";busy=false;sendJson(s);
}
void setup(){
  // Initialisation relais OFF au plus tot. Attention GPIO0/D3 et GPIO2/D4 sont des broches de boot.
  digitalWrite(RELAIS_ONOFF,HIGH);digitalWrite(RELAIS_V1,HIGH);digitalWrite(RELAIS_V2,HIGH);digitalWrite(RELAIS_V3,HIGH);
  pinMode(RELAIS_ONOFF,OUTPUT);pinMode(RELAIS_V1,OUTPUT);pinMode(RELAIS_V2,OUTPUT);pinMode(RELAIS_V3,OUTPUT);
  pinMode(FLOW_PIN,INPUT);vitesseActuelle=0;
  initConfig();beginModbus();
  WiFi.mode(WIFI_STA);WiFi.begin(WIFI_SSID,WIFI_PASS);
  server.on("/",HTTP_GET,handleRoot);
  server.on("/pompe/on",HTTP_GET,[]{if(busy){server.send(409,"text/plain","busy");return;}pompeVitesse(1);server.send(200,"text/plain","Pompe ON (Vitesse 1)");});
  server.on("/pompe/off",HTTP_GET,[]{if(busy){server.send(409,"text/plain","busy");return;}pompeArret();server.send(200,"text/plain","Pompe OFF");});
  server.on("/pompe/vitesse",HTTP_GET,[]{if(busy){server.send(409,"text/plain","busy");return;}uint32_t v;if(!parseUint("v",v,1,3)){server.send(400,"text/plain","Parametre ?v=1|2|3 requis");return;}pompeVitesse(v);server.send(200,"text/plain",String("Pompe ON (Vitesse ")+v+")");});
  server.on("/pompe/status",HTTP_GET,handlePompeStatus);
  server.on("/pompe/flow",HTTP_GET,handleFlowStatus);
  server.on("/astral/status",HTTP_GET,handleAstralStatus);
  server.on("/astral/config",HTTP_GET,handleConfig);
  server.on("/astral/read",HTTP_GET,handleRead);
  server.on("/astral/scan",HTTP_GET,handleScan);
  server.onNotFound([](){sendJson("{\"error\":\"Route inconnue\"}",404);});
  server.begin();
}
void loop(){server.handleClient();if(WiFi.status()!=WL_CONNECTED){static uint32_t last=0;if(millis()-last>10000){last=millis();WiFi.reconnect();}}}
