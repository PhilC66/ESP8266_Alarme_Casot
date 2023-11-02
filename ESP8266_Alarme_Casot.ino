/*
Systeme d'Alarme
Philippe CORBEL 21/12/2017

version en service depuis 10/02/2018

Attention ArduinoJson derniere version 6 necessite modification 
voir https://arduinojson.org/v6/doc/upgrade/?utm_source=github&utm_medium=release

V1-6 ajout enregistrement en EEPROM pas installé

todo
enregistrement EEPROM???

gestion fine des messages retour de surveillance.php
led info ne fonctionne pas bien?
message remonté DesActivation Alarme n'affiche pas RX433, c'est ok à l'activation?
*/

/*
	02/11/2023 testé OK
	WiFiManager V2.0.16-rc.2
	parametrage WiFiManager timeout et retry
	Compilation esp8266:esp8266:huzzah,default,80MHz,4M(3M SPIFFS) ESP8266 2.5.2
  Arduino IDE 1.8.19 : 424256 40%, 38044 46% sur PC
	
	01/11/2023 testé OK
	ajouté WiFiManager
	Compilation esp8266:esp8266:huzzah,default,80MHz,4M(3M SPIFFS) ESP8266 2.5.2
  Arduino IDE 1.8.19 : 391140 37%, 40940 49% sur PC

	22/10/2023 pas encore testé
	suppression json remplacé par code propre
	Compilation esp8266:esp8266:huzzah,default,80MHz,4M(3M SPIFFS) ESP8266 2.5.2
  Arduino IDE 1.8.19 : 375308 35%, 36392 44% sur PC
*/

/*
 ports utilisés :
  0,  Switch GPIO 0 Led Rouge
  2,  o/p signal de vie LedBleue , Boot mode detect, Led Blue utiliser seulement en sortie
  4,  Ip telecommande 433
  5,  Op Buzzer
  12, Op Sirene
  13, Ip Pir
  14, Ip Alarme Secteur
  15, Voyant detection PIR, Boot mode detect,R pull down pour garantir 0 au demarrage, utilisable en sortie/
  16, connexion RST wake up deep sleep
*/


// extern "C" {
// #include "user_interface.h" // this is for the RTC memory read/write functions, pas util??
// }

#include <credentials_home.h>     // informations de connexion Wifi
#include <ESP8266WiFi.h>          // Biblio Wifi
#include <WiFiUdp.h>							// Upload Wifi
#include <ArduinoOTA.h>						// Upload Wifi
#include <RemoteDebug.h>					// Telnet Debug
#include <ESP8266WebServer.h>			// Serveur Httpp
#include <TimeLib.h>							// gestion Heure
#include <Time.h>									// gestion Heure
#include <TimeAlarms.h>						// gestion des Alarmes
#include <Timezone.h>							// gestion DST
#include <ESP8266httpUpdate.h>    // Update Over The Air
#include <EEPROM.h>								// variable en EEPROM
#include <WiFiManager.h>          // Helps with connecting to Wifi
// #include <ArduinoJson.h>
// #include <JsonStreamingParser.h>
#include <RCSwitch.h>


ESP8266WebServer server(80);    // On instancie un serveur sur le port 80
WiFiClient client;
RemoteDebug Debug;

#define LedPIR		15
#define LedBleue	2
#define OpSirene	12
#define OpBuzzer	5
#define	Ip433			4
#define	Ip_PIR		13
#define	IpSecteur	14

const String soft = "ESP8266_Alarme_Casot.ino.adafruit"; 	// nom du soft
String  ver = "V1-6";

struct  config_t 										// Structure configuration sauvée en EEPROM
{
  int 		magic		;									// num magique
  long  	Ala_Vie ;									// Heure message Vie, 8h matin en seconde = 8*60*60
  bool    Intru   ;									// Alarme Intrusion active
  bool    Silence ;									// Mode Silencieux = true false par defaut  
  int 		Dsonn 	;									// Durée Sonnerie
  int 		DsonnMax;									// Durée Max Sonnerie
  int 		Dsonnrepos;								// Durée repos Sonnerie
	int 		Jour_TmCptMax;	 					// Jour Temps de la boucle fausses alarme en s
	int 		Jour_Nmax ;				        // Jour Nombre de fausses alarmes avant alarme
	int 		Nuit_TmCptMax;						// Nuit Temps de la boucle fausses alarme en s
	int 		Nuit_Nmax ;				        // Nuit Nombre de fausses alarmes avant alarme	
	bool    IntruAuto;								// Mode Alarme Intrusion automatique entre IntruDebut et IntruFin
	long 		IntruFin;									// Heure arret Alarme Intru Matin parametre jour/nuit
	long 		IntruDebut;								// Heure debut Alarme Intru Soir
	int     CoeffTension;							// Coefficient calibration Tension
	bool    Abs;											// Absence = 1, active automatiquement IntruON et desactive RX433
} ;
config_t config;
String Message;											//  message qui sera interpreté
String fl = "\n";										//	saut de ligne SMS
String Reponse = "";
char JSONmessageBuffer[600];

byte confign						= 0;				// Num enregistrement EEPROM
byte CptTest						= 12;				// décompteur en mode test si=0 retour tempo normale

int	CoeffTensionDefaut	= 3100;			// Coefficient par defaut
int TmCptMax					 	= 0;				// Temps de la boucle fausses alarme
int Nmax								= 0;				// Nombre de fausses alarmes avant alarme
unsigned int timecompte	= 0;				// comptage nbr passage dans loop compteur temps fausses alarmes
unsigned int FausseAlarme	= 0;			// compteur fausse alarme 

volatile unsigned long rebond = 0;	//	antirebond IRQ
volatile int CptAlarme	= 0;				//	compteur alarme avant filtrage M1

boolean FirstSonn 			= false;		// Premier appel sonnerie
boolean SonnMax   			= false;		// temps de sonnerie maxi atteint
boolean FlagCalibration = false;		// Calibration Tension en cours
boolean FlagPIR 				= false;		// detection PIR
boolean FlagTempoIntru 	= false;		// memorise config.Intru au demarrage
boolean FlagAlarmeIntrusion			= false;	//	Alarme Intrusion detectée
boolean FlagLastAlarmeIntrusion = false;
boolean FlagAlarmeTension 			= false;	//	Alarme tension Batterie
boolean FlagLastAlarmeTension		= false;
boolean FlagAlarmeSect 					= false;	//	Alarme Secteur
boolean FlagLastAlarmeSect 			= false;
boolean FlagReset								=	false;  // reset demandé
boolean FlagDebut								= true;		// reste true jusqu'au message lancement

float 	TensionBatterie;		  						// tension Batterie
unsigned long timer;											// timer pour LedBleue signe de vie

// NTP Servers:
static const char ntpServerName[] = "fr.pool.ntp.org";
int timeZone = 2;     							// Central European Time, heure sans DST

WiFiUDP Udp;
unsigned int 	localPort = 8888;  							// local port to listen for UDP packets
const int 		NTP_PACKET_SIZE = 48;          	// NTP time is in the first 48 bytes of message
byte      		packetBuffer[NTP_PACKET_SIZE];	// buffer to hold incoming & outgoing packets

//Central European Time (Frankfurt, Paris) definition DST
TimeChangeRule CEST = {"CEST", Last, Sun, Mar, 2, 120};     //Central European Summer Time
TimeChangeRule CET 	= {"CET" , Last, Sun, Oct, 3, 60};      //Central European Standard Time
Timezone CE(CEST, CET);

RCSwitch mySwitch = RCSwitch();								//	Rx 433

/* Identification des Alarmes*/
AlarmId FirstMessage;		// 0 tempo lancement premier message et activation Alarme au demarrage
AlarmId loopPrincipale;	// 1 boucle principlae
AlarmId Svie;						// 2 tempo Signal de Vie
AlarmId MajH;						// 3 tempo mise à l'heure régulière
AlarmId TSonn;					// 4 tempo durée de la sonnerie
AlarmId TSonnMax;				// 5 tempo maximum de sonnerie
AlarmId TSonnRepos;			// 6 tempo repos apres maxi
AlarmId HIntruF;				// 7 Heure Fin Matin parametre
AlarmId HIntruD;				// 8 Heure Debut Soir parametre

/* commande serie */
char 		receivedChar;
boolean newData = false;
String 	MessageSerie;
/* commande serie */
//---------------------------------------------------------------------------
ICACHE_RAM_ATTR void IRQ_PIR() {				// Detection PIR
  
		if (millis() - rebond > 50){	// Antirebond
			CptAlarme ++;
			rebond = millis();
		}
}
//---------------------------------------------------------------------------
void configModeCallback (WiFiManager *myWiFiManager);
//---------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
	// /* WiFiManager */
  WiFiManager wifiManager;
  /* Uncomment for testing wifi manager */
  // wifiManager.resetSettings();
  wifiManager.setAPCallback(configModeCallback);
	wifiManager.setConfigPortalTimeout(120); // sets timeout before AP,webserver loop ends and exits
  wifiManager.setConnectRetries(10);       // sets number of retries for autoconnect
  /* or use this for auto generated name ESP + ChipID */
  if (!wifiManager.autoConnect()) {
    Serial.println(F("failed to connect and hit timeout"));
    delay(1000);
    //reset and try again, or maybe put it to deep sleep
    ESP.reset();
    delay(1000);
  }
	/* Manual Wifi */
  // WiFi.begin(mySSID, myPASSWORD);
  // Serial.println("");
  // // on attend d'etre connecte au WiFi avant de continuer
  // while (WiFi.status() != WL_CONNECTED) {
  //   delay(500);
  //   Serial.print(".");
  // }

  // on affiche l'adresse IP qui nous a ete attribuee
  Serial.println("");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
	Udp.begin			(localPort);
	ArduinoOTA.setHostname("ESP_AlarmeCasot"); 	// on donne une nom a notre module
  ArduinoOTA.begin(); 									// initialisation de l'OTA
	Debug.begin("ESP_AlarmeCasot"); 
	Debug.setResetCmdEnabled(true);				// autorise reset par telnet
	mySwitch.enableReceive(Ip433);				// initialisation RX433
  // on definit les points d'entree (les URL a saisir dans le navigateur web) et on affiche un simple texte
	server.on ("/", handleClient);
  
  server.begin(); // on demarre le serveur web
	
  Serial.print(F("Version Soft : ")),Serial.println(ver);
	
	/* Lecture configuration en EEPROM	 */
  EEPROM.begin(512);	
	EEPROM.get(confign, config);
  if (config.magic != 01234) {
    /* verification numero magique si different
				erreur lecture EEPROM ou carte vierge
    		on charge les valeurs par défaut */
    Serial.println(F("Nouvelle carte vierge !"));
    config.magic 				 = 01234;
    config.Ala_Vie 			 = 28800;
    config.Intru 				 = false;
    config.Silence			 = true;
    config.Dsonn				 = 60;
    config.DsonnMax			 = 90;
    config.Dsonnrepos    = 120;
		config.Jour_TmCptMax = 60;
		config.Jour_Nmax		 = 2;
		config.Nuit_TmCptMax = 60;
		config.Nuit_Nmax		 = 2;
		config.IntruAuto		 = true;			
		config.IntruFin		 	 = 25200; 		// 07h00
		config.IntruDebut		 = 75600; 		// 21h00
		config.Abs					 = 0;			
		config.CoeffTension  = CoeffTensionDefaut;			// valeur par defaut
		EEPROM.put(confign,config); // V1-6 bug ligne manquante 
	}
	EEPROM.end();
	// pas de parametres nuit different de jour
	config.Nuit_Nmax = config.Jour_Nmax;
	config.Nuit_TmCptMax = config.Jour_TmCptMax;
	
	PrintEEPROM();
	
	TensionBatterie = map(moyenneAnalogique(),0,1023,0,config.CoeffTension);
	
	/* Mise à l'heure boucle si pas de réponse */
	do {
    byte cpt = 0;
		MajHeure();
    delay(200);
		cpt ++;
		if(cpt > 100)break;								// on sort si 100 tentatives infructueuses, prochaine tentative chaque heure
  } while (timeStatus() == 0);
	AIntru_HeureActuelle(); 						// armement Alarme selon l'heure
	
	if (config.Intru) {									// si Alarme Intru active
    FlagTempoIntru = config.Intru;		// on memorise
    config.Intru = false;							// on desactive jusqu'a tempo demarrage 1mn
  }	
	
  pinMode(LedPIR   , OUTPUT);
	pinMode(LedBleue , OUTPUT);
	pinMode(OpSirene , OUTPUT);
	pinMode(OpBuzzer , OUTPUT);
	pinMode(Ip_PIR	 , INPUT_PULLUP);
	pinMode(IpSecteur, INPUT_PULLUP);
	pinMode(Ip433		 , INPUT);
	
	digitalWrite(LedPIR	  ,HIGH);
	digitalWrite(LedBleue	,HIGH);
	digitalWrite(OpSirene	,LOW);
	digitalWrite(OpBuzzer	,LOW);	
	
 /* parametrage des Alarmes */

  FirstMessage = Alarm.timerOnce(30, OnceOnly); // appeler une fois apres 30 secondes type=0
  
  loopPrincipale = Alarm.timerRepeat(15, Acquisition); // boucle principale 15s
  Alarm.enable(loopPrincipale);
  
  MajH=Alarm.timerRepeat(3600, MajHeure);						// toute les heures  type=1
  Alarm.enable(MajH);

  TSonn = Alarm.timerRepeat(config.Dsonn, ArretSonnerie);		// tempo durée de la sonnerie
  Alarm.disable(TSonn);

  TSonnMax = Alarm.timerRepeat(config.DsonnMax, SonnerieMax); // tempo maximum de sonnerie
  Alarm.disable(TSonnMax);

  TSonnRepos = Alarm.timerRepeat(config.Dsonnrepos, ResetSonnerie); // tempo repos apres maxi
  Alarm.disable(TSonnRepos);

  Svie = Alarm.alarmRepeat(config.Ala_Vie, SignalVie); // chaque jour type=3
  
  Alarm.enable(Svie);
	
	HIntruF = Alarm.alarmRepeat(config.IntruFin, IntruF);
	HIntruD = Alarm.alarmRepeat(config.IntruDebut , IntruD);	
	Alarm.enable(HIntruD);
	Alarm.enable(HIntruF);
	
  // Serial.print(F("FreeRAM = ")), Serial.println(freeRam());
	rebond = millis();
}	//fin setup

//---------------------------------------------------------------------------
void loop() {
	/* commande serie */
  recvOneChar();
  showNewData();
  /* commande serie */
	long value = 0;	// RX433
	
	if(rebond > millis()) rebond = millis();
	static boolean timerlance=false;						//	activation timer alarme 1mn

	if (config.Intru && CptAlarme > 0){	
		if (map(moyenneAnalogique(), 0, 1023, 0, config.CoeffTension) > 1150){ // prise en compte seulement si Vbatt OK, fausse alarme si Vbatt coupée	V2-122
			if (!timerlance){
				timerlance = true;				// on lance le timer si pas deja fait
				timecompte = millis();
			}
			// Serial.print(F("timecompte : ")), Serial.print(timecompte);
			// Serial.print(F("Diff  : ")), Serial.println((millis()-timecompte)/1000);
			
			if ((CptAlarme > Nmax) && (millis() - timecompte < TmCptMax*1000)){		// Alarme validée
				FlagPIR = true;
			}

			if ((millis() - timecompte > TmCptMax*1000) || FlagPIR){ 	// remise à 0 du comptage apres delai ou alarme detectée		V2-122					
				timerlance = false; 									// on arrete le timer
				timecompte = 0;
				FausseAlarme  += CptAlarme;	
				CptAlarme  = 0;	
				Serial.print(F("fausse alarmes : ")), Serial.println(FausseAlarme);
				Acquisition();	// on lance Sirene et SMS directement sans attendre prochaine boucle
			}
		}
		else {
			CptAlarme = 0; // si batterie coupée efface alarme
		}	
	}	
  
	if (mySwitch.available()) {														// RX433    
		value = mySwitch.getReceivedValue();    
    if (value == 0){
      Serial.print(F("Unknown encoding"));
    }
		else{
			if(value == 5592512){															// A = Armement
				Message = F("INTRUON");
				InterpreteMessage(2);
			}
			else if(value == 5592368){ 												// B = Desarmement
				Message = F("INTRUOFF");
				InterpreteMessage(2);
			}
		}
		mySwitch.resetAvailable();
	}
	
  if ( digitalRead(Ip_PIR)) {
    digitalWrite(LedPIR, HIGH);	// allume led locale temoin alarme
  }
  else {
    digitalWrite(LedPIR, LOW);
  }

	ledcligno();
	ArduinoOTA.handle();	// Telechargement en Wifi 
	Debug.handle();				// Debug par Telnet
  server.handleClient();// pour que les requetes soient traitees en Http
	Alarm.delay(10);
}
//---------------------------------------------------------------------------
void Acquisition(){
  //	boucle acquisition 15s
  Serial.println(displayTime(0));	//displayTime(false);
	Debug.println(displayTime(0));
	
  static byte nalaTension = 0;
	static byte nRetourTension = 0; //V1-16
  TensionBatterie = map(moyenneAnalogique(), 0, 1023, 0, config.CoeffTension);//3088, X4573=3029, X4545=3128 V2-122
	// Debug.print(F("Tension batterie = ")), Debug.println(TensionBatterie);
	// Regulateur Solaire coupe à 11.2V
  if (TensionBatterie < 1162 ) {// V1-16 V1-12
    nalaTension ++;
    if (nalaTension == 4) {
      FlagAlarmeTension = true;
      nalaTension = 0;
    }
  }
	else if (TensionBatterie > 1242) {	// hysteresis et tempo sur Alarme Batterie
    nRetourTension ++;
		if(nRetourTension == 4){
			FlagAlarmeTension = false;				
			nRetourTension =0;
		}
  }
  else {
    if (nalaTension > 0)nalaTension--;			//	efface progressivement le compteur
  }
	
	static byte nalaSecteur = 0;
  if (digitalRead(IpSecteur)) {
    nalaSecteur ++;
    if (nalaSecteur == 4) {
      FlagAlarmeSect = true;
      nalaSecteur = 0;
    }
    Serial.println(F("Alarme Secteur"));
  }
  else {
    FlagAlarmeSect = false;
    if (nalaSecteur > 0)nalaSecteur--;			//	efface progressivement le compteur
  }	

	static byte nalaPIR = 0;
  if (config.Intru) { 
		// gestion des capteurs coupé ou en alarme permanente
		// verif sur 3 passages consecutifs
		if (digitalRead(Ip_PIR)){
			nalaPIR ++;
			if(nalaPIR > 3){
				CptAlarme = 1;
				FausseAlarme = 1000;
				FlagPIR = true;
				nalaPIR = 0;
			}
		}
		else{
			if (nalaPIR > 0) nalaPIR --;			//	efface progressivement le compteur
		}
		
		// fin gestion des capteurs coupé en alarme permanente	
		
		if (TensionBatterie > 1150){	// V1-12 seulement si Batterie OK
			if(FlagPIR) {	
				FlagAlarmeIntrusion = true;	// Si alarme intrusion active et intrusion detectée
				FlagPIR = false;
				ActivationSonnerie();			// activation Sonnerie
				Serial.println(F("Alarme Intrusion"));			
			}
     Serial.print(F("CptAlarme = ")), Serial.print(CptAlarme);
			if(CptAlarme){
			 Serial.print(F(", Compte tempo = ")),Serial.println((millis()-timecompte)/1000);
			}
			else{
				Serial.println();
			}	
		}
  }
	else{
		FlagPIR = false;	// efface alarme pendant phase de démarrage
		CptAlarme = 0;
	}

  envoie_alarme();
	
	if (FlagReset)ESP.restart();					//	redemarrage ESP8266

  // Serial.print(F("freeRAM=")),Serial.println(freeRam());
	// Serial.print(F( "CptAlarme = ")), Serial.println(CptAlarme);
}	
//---------------------------------------------------------------------------
void envoie_alarme() {
  /* determine si un message appartition/disparition Alarme doit etre envoyé */
  boolean SendEtat = false;
	
  if (FlagAlarmeTension != FlagLastAlarmeTension) {
    SendEtat = true;
    FlagLastAlarmeTension = FlagAlarmeTension;
		
		if(FlagAlarmeTension){
			Reponse = F("Alarme Tension");
			Debug.println(F("Alarme Tension"));
		}
		else{
			Reponse = F("Fin Alarme Tension");
		}
  }
	if (FlagAlarmeSect != FlagLastAlarmeSect) {
    SendEtat = true;
		FlagLastAlarmeSect = FlagAlarmeSect;
		
		if(FlagAlarmeSect){
			Reponse = F("Alarme Secteur");
		}
		else{
			Reponse = F("Fin Alarme Secteur");
		}
  }
  if (FlagAlarmeIntrusion != FlagLastAlarmeIntrusion) {
    SendEtat = true;
		FlagLastAlarmeIntrusion = FlagAlarmeIntrusion;
		
		if(FlagAlarmeIntrusion){
			Reponse = F("Alarme Intrusion");
		}
		else{
			Reponse = F("Fin Alarme Intrusion");
		}
  }
  if (SendEtat) { 							// si envoie Etat demandé
		CreateJson();
		rw_data(0);	// envoie du json sur bdd
		// Serial.print("envoi du buffer"), Serial.println(JSONmessageBuffer);
		
		SendEtat = false;						// efface demande
  }
}
//---------------------------------------------------------------------------
void InterpreteMessage(byte nbr){
	// nbr = 99 demande locale série
	// nbr = 0  Automatique
	// nbr = 1  demande page web
	// nbr = 2	RX433
	Debug.print("message avant interpretation = "),Debug.println(Message);
	Reponse = "";
	static int tensionmemo = 0;	//	memorisation tension batterie lors de la calibration V2-122
	if(nbr == 2 && config.Abs) { // si reception RX433 en Absence c'est anormal, on sort!
		Reponse  = "Rx433 en Absence?, ";
		Reponse += Message;
		goto fin;
	}
	else if(Message.indexOf(F("TIME")) == 0) {
		Serial.println(displayTime(0));		
	}
	else if (Message.indexOf(F("INTRU")) == 0 ) {		//	Alarme Intrusion
		if (Message.indexOf(F("ON")) == 5) {
			if(nbr == 2)Buzzer(2);
			if (!config.Intru) {
				config.Intru = !config.Intru;
				config.IntruAuto = false;
				sauvConfig();															// sauvegarde en EEPROM
				ActiveAlarme();
			}
		}
		else if (Message.indexOf(F("OFF")) == 5) {
			if(nbr == 2)Buzzer(1);
			if (config.Intru) {	//	|| config.IntruAuto
				config.Intru 		 = false;
				sauvConfig();															// sauvegarde en EEPROM
				DesActiveAlarme();
			}
		}
		if (Message.indexOf(F("AUTO")) == 5) {
			if(Message.indexOf(F("ON")) == 9){
				if(!config.IntruAuto){
					config.IntruAuto = true;
					sauvConfig();															// sauvegarde en EEPROM
					AIntru_HeureActuelle(); // armement selon l'heure
				}
			}
			else if(Message.indexOf(F("OFF")) == 9){
				if(config.IntruAuto){
					config.IntruAuto = false;
					sauvConfig();															// sauvegarde en EEPROM
					AIntru_HeureActuelle(); // armement selon l'heure
				}
			}
		}
		if(config.Intru){
				Reponse += F("Alarme Intrusion ON");	 //Alarme Intrusion ON
			}
			else{
				Reponse += F("Alarme Intrusion OFF"); //Alarme Intrusion OFF
		}
		if(config.IntruAuto){
			Reponse += F("/Auto ON");	 		//Alarme Intrusion Auto
		}
		else{
			Reponse += F("/Auto OFF");	 //Alarme Intrusion Auto
		}
	}
	else if (Message.indexOf(F("ABSENCE")) == 0 ) {		//	Alarme en Absence
		if (Message.indexOf(F("ON")) == 7) {			
			if (!config.Abs) {
				config.Abs				= true;
				config.IntruAuto	= false;
				config.Intru			= true;
				config.Silence		= false;
				sauvConfig();															// sauvegarde en EEPROM
				ActiveAlarme();
			}
		}
		else if (Message.indexOf(F("OFF")) == 7) {			
			if (config.Abs) {
				config.Abs				= false;
				config.IntruAuto	= true;
				config.Intru			= false;				
				sauvConfig();															// sauvegarde en EEPROM
				DesActiveAlarme();
			}
		}
		Reponse = F("Absence ");
		if (config.Abs){
			Reponse += F("ON");
		}
		else{
			Reponse += F("OFF");
		}
	}
	else if (Message.indexOf(F("HINTRU")) == 0 || Message.indexOf(F("HPARAM")) == 0 ) {		//V2-122	Heures chagement parametre
		if (Message.indexOf(char(61)) == 6) {	//	"=" changement heure Intru Auto
			// Hintru=Hsoir,Hmatin; Hintru=75600,21600
			int  x = Message.indexOf(",");
			long i = atol(Message.substring(7, x).c_str());		// valeur Soir
			long j = atol(Message.substring(x+1).c_str());		// valeur Matin	
			if (i > 0 && i <= 86340 && 
					j > 0 && j <= 86340);{ //	ok si i entre 0 et 86340(23h59) et > Heure matin 	ok si j entre 0 et 86340(23h59) et < Heure soir
				if(config.IntruDebut != i || config.IntruFin != j){// si changement
					config.IntruDebut 	= i;
					config.IntruFin 		= j;
					
					sauvConfig();							// sauvegarde en EEPROM
					Alarm.disable(HIntruD);		// on arrete les alarmes
					Alarm.disable(HIntruF);						
					HIntruF = Alarm.alarmRepeat(config.IntruFin, IntruF);// on parametre
					HIntruD = Alarm.alarmRepeat(config.IntruDebut , IntruD);	
					Alarm.enable(HIntruD);		// on redemarre les alarmes
					Alarm.enable(HIntruF);
					AIntru_HeureActuelle();
				}
			}
		}
		Reponse += F("Auto ");
		// Reponse += fl;
		Reponse += F("debut: ");
		Reponse += ChngFormatAlarme(config.IntruDebut);
		// Reponse += int(config.IntruDebut / 3600);
		// Reponse += ":";
		// Reponse += int((config.IntruDebut % 3600) / 60);
		// Reponse += F("(hh:mm)");
		// Reponse += fl;
		Reponse += F(", fin : ");
		Reponse += ChngFormatAlarme(config.IntruFin);
		// Reponse += int(config.IntruFin / 3600);
		// Reponse += ":";
		// Reponse += int((config.IntruFin % 3600) / 60);
		// Reponse += F("(hh:mm)");		
	}
	else if (Message.indexOf(F("SILENCE")) == 0 ) {		//	Alarme Silencieuse
		if (Message.indexOf(F("ON")) == 7) { //ON
			if (!config.Silence) {
				config.Silence = !config.Silence;
				sauvConfig();															// sauvegarde en EEPROM
			}
		}
		if (Message.indexOf(F("OFF")) == 7) {
			if (config.Silence) {
				config.Silence = !config.Silence;
				sauvConfig();															// sauvegarde en EEPROM
				/*	Arret Sonnerie au cas ou? sans envoyer SMS */
				digitalWrite(OpSirene, LOW);	// Arret Sonnerie
				Alarm.disable(TSonn);			// on arrete la tempo sonnerie
				Alarm.disable(TSonnMax);	// on arrete la tempo sonnerie maxi
			}
		}
		if(config.Silence){
			Reponse += F("Silence ON");
		}
		else{
			Reponse += F("Silence OFF");
		}			
	}
	else if (Message.indexOf(F("PARAM")) == 0){				//	Parametres fausses alarmes
		if (Message.indexOf(char(61)) == 5){
			int x = Message.indexOf(":");
			// int y = Message.indexOf(":",x+1);
			// int z = Message.indexOf(":",y+1);
			int i = atoi(Message.substring(6, x).c_str());		// Jour nombre de fausses Alarmes
			int j = atoi(Message.substring(x+1).c_str());	// duree analyse					
			// int j = atoi(Message.substring(x+1,y).c_str());	// duree analyse					
			// int k = atoi(Message.substring(y+1, z).c_str());	// Nuit nombre de fausses Alarmes
			// int l = atoi(Message.substring(z + 1).c_str());	// duree analyse
			if (i >= 0 && i < 101 && j > 1 && j < 120){
				// nombre entre 1 et 100, durée entre 1 et 120s
				config.Jour_Nmax 		 = i;
				config.Jour_TmCptMax = j; //passage en n s
				config.Nuit_Nmax = config.Jour_Nmax;
				config.Nuit_TmCptMax = config.Jour_TmCptMax;
				// config.Nuit_Nmax 		 = k;
				// config.Nuit_TmCptMax = l * 10; //passage en n*100ms
				
				sauvConfig();													// sauvegarde en EEPROM V2-12	
				AIntru_HeureActuelle();				
			}
		}				
		Reponse += F("Param Fs Alarmes : ");
		Reponse += F("n = ");				
		Reponse += config.Jour_Nmax;
		Reponse += F(", t = ");				
		Reponse += config.Jour_TmCptMax;
		Reponse += F("(s)");
		// Reponse += fl;
		// Reponse += F("Nuit n = ");				
		// Reponse += config.Nuit_Nmax;
		// Reponse += F(", t = ");				
		// Reponse += config.Nuit_TmCptMax / 10;
		// Reponse += F("(s)");
		// Reponse += fl;
		// Reponse += F("Actuel n = ");				
		// Reponse += Nmax;
		// Reponse += F(", t = ");				
		// Reponse += TmCptMax / 10;
		// Reponse += F("(s)");
	}
	else if(Message.indexOf(F("SIRENE")) == 0){			// Lancement SIRENE
		digitalWrite(OpSirene, HIGH);									// Marche Sonnerie
		Alarm.enable(TSonn);													// lancement tempo
		Reponse += F("Lancement Sirene");
		Reponse += fl;
		Reponse += config.Dsonn;
		Reponse += F("(s)");				
	}
	else if(Message.indexOf(F("BUZZER")) == 0){			// Lancement BUZZER test
		Buzzer(2);
		Reponse += F("Lancement Buzzer");
		Serial.println(Reponse);
		//Reponse = "";
	}
	else if (Message.indexOf(F("ST")) == 0){
		Reponse = F("St");
	}
	else if(Message.indexOf(F("RESET")) == 0 || Message.indexOf(F("RST")) == 0){
		Reponse = F("Reset du système..");
		FlagReset = true;
	}
	else if (Message.indexOf(F("SONN")) == 0) {			//	Durée Sonnerie
		if (Message.indexOf(char(61)) == 4) {
			int x = Message.indexOf(":");
			int y = Message.indexOf(":", x + 1);

			int i = atoi(Message.substring(5, x).c_str());			//	Dsonn Sonnerie
			int j = atoi(Message.substring(x + 1, y).c_str());	//	DsonnMax Sonnerie
			int k = atoi(Message.substring(y + 1).c_str()); 		//	Dsonnrepos Sonnerie
			//Serial.print(i),Serial.print(","),Serial.print(j),Serial.print(","),Serial.println(k);
			if (i > 4  && i <= 300 &&
					j > i  && j <= 600 &&
					k > 9  && k <= 300) {			//	ok si entre 10 et 300
				config.Dsonn 			= i;
				config.DsonnMax 	= j;
				config.Dsonnrepos = k;
				sauvConfig();																// sauvegarde en EEPROM
			}
		}
		Reponse += F("Param Sonnerie = ");
		Reponse += config.Dsonn;
		Reponse += ":";
		Reponse += config.DsonnMax;
		Reponse += ":";
		Reponse += config.Dsonnrepos;
		Reponse += "(s)";
	}
	else if (Message.indexOf(F("VIE")) == 0) {					//	Heure Message Vie
			if ((Message.indexOf(char(61))) == 3) {
				long i = atol(Message.substring(4).c_str()); 	//	Heure Reponse Vie
				Serial.print("hvie "),Serial.println(i);
				if (i > 0 && i <= 86340) {										//	ok si entre 0 et 86340(23h59)
					Serial.print("hvie "),Serial.println(i);	
					config.Ala_Vie = i;
					sauvConfig();																// sauvegarde en EEPROM
					Svie = Alarm.alarmRepeat(config.Ala_Vie, SignalVie);	// init tempo
				}
			}
		Serial.print("config.Ala_Vie = "),Serial.println(config.Ala_Vie);	
		Reponse += F("Heure Vie = ");
		Reponse += ChngFormatAlarme(config.Ala_Vie);
		// Reponse += int(config.Ala_Vie / 3600);
		// Reponse += ":";
		// Reponse += int((config.Ala_Vie % 3600) / 60);
		// Reponse += F("(hh:mm)");		
	}
	else if (nbr==99 && Message.indexOf(F("CALIBRATION=")) == 0){	
		/* 	Mode calibration mesure tension V2-122
				Seulement en mode serie local
				recoit Reponse "CALIBRATION=0000"
				entrer mode calibration
				effectue mesure tension avec CoeffTensionDefaut retourne et stock resultat
				recoit Reponse "CALIBRATION=1250" mesure réelle en V*100
				calcul nouveau coeff = mesure reelle/resultat stocké * CoeffTensionDefaut
				applique nouveau coeff
				stock en EEPROM
				sort du mode calibration

				variables
				FlagCalibration true cal en cours, false par defaut
				static int tensionmemo memorisation de la premiere tension mesurée en calibration
				int config.CoeffTension = CoeffTensionDefaut 3100 par défaut
		*/
		String bidon=Message.substring(12,16);
		//Serial.print(F("bidon=")),Serial.print(bidon),Serial.print(","),Serial.println(bidon.length());
		if(bidon.substring(0,1) == "0" ){// debut mode cal
			FlagCalibration = true;
			config.CoeffTension = CoeffTensionDefaut;
			TensionBatterie = map(moyenneAnalogique(), 0,1023,0,config.CoeffTension);
			// Serial.print("TensionBatterie = "),Serial.println(TensionBatterie);
			tensionmemo = TensionBatterie;					
		}
		
		else if(FlagCalibration && bidon.substring(0,4).toInt() > 0 && bidon.substring(0,4).toInt() <=5000){
			// si Calibration en cours et valeur entre 0 et 5000

			/* calcul nouveau coeff */
			config.CoeffTension = bidon.substring(0,4).toFloat()/float(tensionmemo)*CoeffTensionDefaut;
			// Serial.print("Coeff Tension = "),Serial.println(config.CoeffTension);
			TensionBatterie = map(moyenneAnalogique(), 0,1023,0,config.CoeffTension);
			// Serial.print("TensionBatterie = "),Serial.println(TensionBatterie);	
			FlagCalibration = false;
			sauvConfig();															// sauvegarde en EEPROM	
		}
		Reponse += F("Mode Calib Tension");
		Reponse += fl;
		Reponse += F("TensionBatterie = ");
		Reponse += TensionBatterie;
		Reponse += fl;
		Reponse += F("Coeff Tension = ");
		Reponse += config.CoeffTension;
		Reponse += fl;
		Reponse += F("Batterie = ");
		Reponse += String(Battpct(TensionBatterie));
		Reponse += "%";
		Serial.println(Reponse);
		Reponse = "";
	}
	else{
		Serial.println(F("Message non reconnu !"));	
	}
fin:
	if(Reponse.length() > 0){// on envoie que si message present
		if(nbr == 2) Reponse += F(", Rx433");
		Serial.println(Reponse);
		CreateJson();
		rw_data(nbr);	// envoie du json sur bdd
	}
	// Serial.print("envoi du buffer"), Serial.println(JSONmessageBuffer);
}
//---------------------------------------------------------------------------
void SignalVie(){
	
	Reponse =F("Signal Vie");
	CreateJson();
	rw_data(0);	// envoie du json sur bdd
}
//---------------------------------------------------------------------------
boolean rw_data(byte id){	// envoie data sur site perso
	boolean flagreturn;
	int 	 pos;
	String data;
	String tempo = "Host:";
	tempo += monSitelocal;
	data   = "id=";
	data  += id;
	data  +="&json=";
	data  += JSONmessageBuffer;
		if (client.connect(monSitelocal, 80)) { // REPLACE WITH YOUR SERVER ADDRESS
		client.println("POST /casot/surveillance.php HTTP/1.1");
		client.println (tempo);          // SERVER ADDRESS HERE TOO
		client.println("Content-Type: application/x-www-form-urlencoded");
		client.print("Content-Length: ");
		client.println(data.length());
		client.println();
		client.print(data);
		delay(100);
		// Serial.print(F("length : ")),Serial.println(data.length());
		// Serial.print(F("Envoye au serveur : ")),Serial.println(data);
		
		String req = client.readStringUntil('\r');// 'X','\r' ne renvoie pas tous les caracteres?
		Serial.print("reponse serveur surveillance : "),Serial.println(req);
		pos = req.indexOf("OK");
		if(pos > -1){
			Serial.println("Ecriture bdd OK");
			flagreturn = true;
		}
		else{
			Serial.println("Ecriture bdd KO");
			flagreturn = false;
		}
	}
	if (client.connected()) {
    client.stop();  // DISCONNECT FROM THE SERVER
  }
	return flagreturn;
}
//---------------------------------------------------------------------------
void handleClient(){ 
	// Debug.print("args = "),Debug.println(server.args());
	// Debug.print("argname = "),Debug.println(server.argName(0));
	// Debug.print("argname = "),Debug.println(server.argName(1));
	// Debug.print("arg = "),Debug.println(server.arg(0));
	// Debug.print("arg = "),Debug.println(server.arg(1));
	boolean Send = false;
	if (server.hasArg("data")){		
		if(server.arg(0).substring(0,3) == "VIE"){
			
			Message  = server.arg(0).substring(0,3);
			Message += "=";
			Message += String(Hms_long(server.arg(0).substring(4,13)));
			// Debug.print("message = "),Debug.println(Message);
			Send = true;
		}
		if(server.arg(0).substring(0,5) == "PARAM"){
			Message = server.arg(0);
			// Debug.print("message = "),Debug.println(Message);
			Send = true;
		}
		if(server.arg(0).substring(0,4) == "SONN"){
			Message = server.arg(0);
			// Debug.print("message = "),Debug.println(Message);
			Send = true;
		}
		if(server.arg(0) == "Int0"){
			Message = "INTRUOFF";
			Send = true;
		}
		else if(server.arg(0) == "Int1"){
			Message = "INTRUON";
			Send = true;
		}
		else if(server.arg(0) == "Sir0"){
			Message = "SILENCEOFF";
			Send = true;
		}
		else if(server.arg(0) == "Sir1"){
			Message = "SILENCEON";
			Send = true;
		}
		else if(server.arg(0) == "Auto0"){
			Message = "INTRUAUTOOFF";
			Send = true;
		}
		else if(server.arg(0) == "Auto1"){
			Message = "INTRUAUTOON";
			Send = true;
		}
		else if(server.arg(0) == "Abs1"){
			Message = "ABSENCEON";
			Send = true;
		}
		else if(server.arg(0) == "Abs0"){
			Message = "ABSENCEOFF";
			Send = true;
		}
		else if(server.arg(0) == "st"){
			Message = server.arg(0);
			Message.toUpperCase();
			Send = true;
		}
		else if(server.arg(0) == "buzzer"){
			Message = server.arg(0);
			Message.toUpperCase();
			Send = true;
		}
		else if(server.arg(0) == "sirene"){
			Message = server.arg(0);//"SIRENE";
			Message.toUpperCase();
			Send = true;
		}
		else if(server.arg(0) == "reset"){
			Message = server.arg(0);
			Message.toUpperCase();
			Send = true;
		}
		if (Send){
			server.send ( 200, "text/html", "OK" );
			InterpreteMessage(1);										// envoie premiere partie
		}
		
		if(server.arg(1).substring(0,6) == "HINTRU"){
			Message  = server.arg(1).substring(0,7);
			// Message += "=";
			Message += String(Hms_long(server.arg(1).substring(7,15)));
			Message += ",";
			Message += String(Hms_long(server.arg(1).substring(16,24)));
			// Debug.print("message = "),Debug.println(Message);
			InterpreteMessage(1);										// envoie deuxieme partie
		}
		
	}
	else {
    server.send ( 200, "text/html", "OK" );
  } 
}
//---------------------------------------------------------------------------
String getPage(){
	String page = "<!DOCTYPE html> <html lang='fr'> <meta charset='utf-8' />";
	page += "<body> :-)";
	page += "</body></html>";
	return page;
}
//---------------------------------------------------------------------------
void ledcligno(){
	if(timer > millis())timer = millis();
	if(millis() - timer > 5000){
		timer = millis();
		digitalWrite(LedBleue,LOW);
		digitalWrite(LedPIR  ,HIGH);
		delay(15);
		digitalWrite(LedBleue,HIGH);
		digitalWrite(LedPIR  ,LOW);
	}
}
//---------------------------------------------------------------------------
void PrintEEPROM(){
	Serial.print(F("config.magic = "))				,Serial.println(config.magic);
	Serial.print(F("config.Ala_Vie = "))			,Serial.println(config.Ala_Vie);
	Serial.print(F("config.Intru = "))				,Serial.println(config.Intru);
	Serial.print(F("config.Silence = "))			,Serial.println(config.Silence);
	Serial.print(F("config.Dsonn = "))				,Serial.println(config.Dsonn);
	Serial.print(F("config.DsonnMax = "))			,Serial.println(config.DsonnMax);
	Serial.print(F("config.Dsonnrepos = "))		,Serial.println(config.Dsonnrepos);
	Serial.print(F("config.Jour_TmCptMax = ")),Serial.println(config.Jour_TmCptMax);
	Serial.print(F("config.Jour_Nmax = "))		,Serial.println(config.Jour_Nmax);
	Serial.print(F("config.Nuit_TmCptMax = ")),Serial.println(config.Nuit_TmCptMax);
	Serial.print(F("config.Nuit_Nmax = "))		,Serial.println(config.Nuit_Nmax);
	Serial.print(F("config.IntruAuto = "))		,Serial.println(config.IntruAuto);
	Serial.print(F("config.IntruFin	= "))			,Serial.println(config.IntruFin);
	Serial.print(F("config.IntruDebut = "))		,Serial.println(config.IntruDebut);
	Serial.print(F("config.CoeffTension = "))	,Serial.println(config.CoeffTension);
	Serial.print(F("config.Abs = "))					,Serial.println(config.Abs);
}
//---------------------------------------------------------------------------
void MajHeure(){	
	MessageFaussesAlarmes();
	
	IPAddress ntpServerIP; // NTP server's ip address
  while (Udp.parsePacket() > 0) ; // discard any previously received packets
  
	// Serial.println(F("Transmit NTP Request")); 
  // Serial.print(ntpServerName);
  // Serial.print(": ");
  // Serial.println(ntpServerIP);
	
  WiFi.hostByName(ntpServerName, ntpServerIP);
	sendNTPpacket(ntpServerIP);
  // sendNTPpacket(timeServer);
  uint32_t beginWait = millis();
  while (millis() - beginWait < 2000) {
    int size = Udp.parsePacket();
    if (size >= NTP_PACKET_SIZE) {
      Udp.read(packetBuffer, NTP_PACKET_SIZE);  // read packet into the buffer
      unsigned long secsSince1900;
      // convert four bytes starting at location 40 to a long integer
      secsSince1900 =  (unsigned long)packetBuffer[40] << 24;
      secsSince1900 |= (unsigned long)packetBuffer[41] << 16;
      secsSince1900 |= (unsigned long)packetBuffer[42] << 8;
      secsSince1900 |= (unsigned long)packetBuffer[43];			
      // return secsSince1900 - 2208988800UL ;//+ timeZone * SECS_PER_HOUR;
			unsigned long epoch = secsSince1900 - 2208988800UL ;
			TimeChangeRule *tcr;
			time_t utc;
			utc = epoch;
			
			Serial.println(F("Receive NTP Response :-)"));
			Debug .println(F("Receive NTP Response :-)"));
			Reponse = F(":-) ");
			SetHorloge(CE.toLocal(utc, &tcr));
			Serial.println(displayTime(0));
			// AIntru_HeureActuelle(); 		// armement selon l'heure
			goto fin;
    }
		// Serial.println("fin de if NTP packets :-(");
  }
  Serial.println(F("No NTP Response :-("));
	Debug .println(F("No NTP Response :-("));
	Reponse = F(":-( ");
	// SendMessageHoraire();
	fin:
	SendMessageHoraire();	
}
//---------------------------------------------------------------------------
void SendMessageHoraire(){
	if(config.Intru){ // seulement pendant les periodes sous Alarme
		Reponse += F("Signal horaire");
		if(!FlagDebut){	// ne pas envoyer pendant phase de démarrage
			CreateJson();
			rw_data(0);	// envoie du json sur bdd	
		}
	}
	Reponse = "";	
}
//---------------------------------------------------------------------------
int moyenneAnalogique(){
  // calcul moyenne 10 mesures consécutives
	int moyenne = 0;
  for (int j = 0; j < 10; j++) {
    delay(20);//20
		moyenne += analogRead(A0);	
	// Serial.print(moyenne)		,Serial.print(":");
  }
  moyenne /= 10;
	// Serial.print(moyenne)		,Serial.println("");
	return moyenne;
}
//---------------------------------------------------------------------------
void sendNTPpacket(IPAddress &address){
  // send an NTP request to the time server at the given address
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;     // Stratum, or type of clock
  packetBuffer[2] = 6;     // Polling Interval
  packetBuffer[3] = 0xEC;  // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12]  = 49;
  packetBuffer[13]  = 0x4E;
  packetBuffer[14]  = 49;
  packetBuffer[15]  = 52;
  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:
  Udp.beginPacket(address, 123); //NTP requests are to port 123
  Udp.write(packetBuffer, NTP_PACKET_SIZE);
  Udp.endPacket();
}
//---------------------------------------------------------------------------
void SetHorloge(time_t t){
	// mise à l'heure horloge logiciel
	// arreter toutes les Alarmes avant maj
	// arreter proprement Sonnerie
	Alarm.disable(FirstMessage);
	Alarm.disable(loopPrincipale);
	ArretSonnerie();
	Alarm.disable(TSonnMax);				// mais elles ne sont pas réarmées, elles le seront si nouvelles alarme
	Alarm.disable(TSonnRepos);
	Alarm.disable(MajH);
	Alarm.disable(Svie);
	Alarm.disable(HIntruF);
	Alarm.disable(HIntruD);
	
	setTime(hour(t), minute(t), second(t), day(t), month(t), year(t));
	
	Alarm.enable(FirstMessage);
	Alarm.enable(loopPrincipale);
	Alarm.enable(MajH);
	Alarm.enable(Svie);
	Alarm.enable(HIntruF);
	Alarm.enable(HIntruD);
}
//---------------------------------------------------------------------------
String displayTime(byte n) {
	// n = 0 ; dd/mm/yyyy hh:mmhss
	// n = 1 ; yyyy-mm-dd hh:mmhss

  String dt;
		if(n == 0){
			if (day() < 10) {
			dt += "0";
			}
			dt += day();
			dt += ("/");
			if (month() < 10) {
				dt += "0";
			}
			dt += month();
			dt += ("/");
			dt += year();			
		}
		else if(n == 1){
			dt += year();
			dt += ("-");
			if (month() < 10) {
				dt += "0";
			}
			dt += month();
			dt += ("-");
			if (day() < 10) {
			dt += "0";
			}
			dt += day();
		}
		dt += (" ");
		if (hour() < 10) {
			dt += "0";
		}
		dt += hour();
		dt += ":";
		if (minute() < 10) {
			dt += "0";
		}
		dt += minute();
		dt += ":";
		if (second() < 10) {
			dt += "0";
		}
		dt += second();
		return dt;
}
//---------------------------------------------------------------------------
void OnceOnly() {
  // Serial.println(Alarm.getTriggeredAlarmId());
  if (FlagTempoIntru) {			//	si Alarme Intru était demandée
    config.Intru = true;		//	on reactive
		ActiveAlarme();
  }
	Reponse = F("Lancement");
	CreateJson();
	rw_data(0);	// envoie du json sur bdd
	FlagDebut = false;
}
//---------------------------------------------------------------------------
void ActivationSonnerie() {
  // Sirène
  if (!SonnMax) {									// pas atteint Temps sonnerie maxi
    if (!config.Silence) digitalWrite(OpSirene, HIGH);		// Marche Sonnerie sauf Silence
    Alarm.enable(TSonn);
    if (!FirstSonn) {							// premiere sonnerie on lance Tempo Max
      Alarm.enable(TSonnMax);
      FirstSonn = true;
    }
  }
}
//---------------------------------------------------------------------------
void ArretSonnerie() {
  Serial.print(F("Fin tempo Sonnerie : "));
  Serial.println(Alarm.getTriggeredAlarmId());
  digitalWrite(OpSirene, LOW);	// Arret Sonnerie
  Alarm.disable(TSonn);			// on arrete la tempo sonnerie
  Alarm.disable(TSonnMax);	// on arrete la tempo sonnerie maxi
  FirstSonn = false;
  FlagAlarmeIntrusion = false;
  FlagPIR = false;
}
//---------------------------------------------------------------------------
void SonnerieMax() {
  Serial.print(F("Fin periode Max Sonnerie : "));
  Serial.println(Alarm.getTriggeredAlarmId());
  // fin de la tempo temps de sonnerie maxi
  Alarm.enable(TSonnRepos);	// on lance la tempo repos sonnerie
  Alarm.disable(TSonnMax);	// on arrete la tempo sonnerie maxi
  Alarm.disable(TSonn);			// on arrete la tempo sonnerie
  digitalWrite(OpSirene, LOW);	// Arret Sonnerie
  FirstSonn = false;///
  SonnMax = true;						// interdit nouveau lancement sonnerie 
														// avant fin tempo repos sonnerie
}
//---------------------------------------------------------------------------
void ResetSonnerie() {
  Serial.print(F("Fin periode inhibition sonnerie : "));
  Serial.println(Alarm.getTriggeredAlarmId());
  // fin de la tempo repos apres temps sonnerie maxi
  Alarm.disable(TSonnRepos);// on arrete la tempo repos
  SonnMax = false;
  ArretSonnerie();
}
//---------------------------------------------------------------------------
void IntruF(){// Charge parametre Alarme Intrusion Jour et Arret Intru
	Nmax 			= config.Jour_Nmax;
	TmCptMax  = config.Jour_TmCptMax;
	Serial.println(F("Fin periode intru"));
	if (config.IntruAuto){
		if(config.Intru){
			config.Intru = false;	//  Alarme OFF	
			DesActiveAlarme();
		}
	}
}
//---------------------------------------------------------------------------
void IntruD(){// Charge parametre Alarme Intrusion Nuit et Active Intru
	Nmax 			= config.Nuit_Nmax;
	TmCptMax  = config.Nuit_TmCptMax;
	Serial.println(F("Debut periode intru"));
	if (config.IntruAuto){
		if(!config.Intru){
			config.Intru = true;	//  Alarme ON	
			ActiveAlarme();
		}
	}	
}
//---------------------------------------------------------------------------
void ActiveAlarme(){
	attachInterrupt(digitalPinToInterrupt(Ip_PIR), IRQ_PIR, RISING);
	CptAlarme 	 = 0;
	FausseAlarme = 0;
	Serial.print(F("Intru = ")),Serial.println(config.Intru);
	Reponse = F("Activation Alarme");
	CreateJson();	// creation du json
	rw_data(0);		// envoie du json sur bdd
}
//---------------------------------------------------------------------------
void DesActiveAlarme(){
	detachInterrupt(digitalPinToInterrupt(Ip_PIR));
	/*	Arret Sonnerie au cas ou? sans envoyer SMS */
	digitalWrite(OpSirene, LOW);	// Arret Sonnerie
	Alarm.disable(TSonn);			// on arrete la tempo sonnerie
	Alarm.disable(TSonnMax);	// on arrete la tempo sonnerie maxi
	FirstSonn = false;
	FlagAlarmeIntrusion = false;	
	Serial.print(F("Intru = ")),Serial.println(config.Intru);
	Reponse = F("DesActivation Alarme");
	CreateJson();
	rw_data(0);	// envoie du json sur bdd
}
//---------------------------------------------------------------------------
void sauvConfig() {
  // Sauvegarde config en EEPROM
  EEPROM.begin(512);	
	EEPROM.put(confign, config);
	EEPROM.end();
	EEPROM.begin(512);	
	EEPROM.get(confign, config);
	EEPROM.end();
}

//---------------------------------------------------------------------------
void AIntru_HeureActuelle(){ //	V2-122
	// chagement parametre intru en fonction de l'heure actuelle
	long Heureactuelle = hour()*60;// calcul en 4 lignes sinon bug!
	Heureactuelle += minute();
	Heureactuelle  = Heureactuelle*60;
	Heureactuelle += second(); // en secondes
	
		if(config.IntruDebut > config.IntruFin){
			if((Heureactuelle > config.IntruDebut && Heureactuelle > config.IntruFin)
			 ||(Heureactuelle < config.IntruDebut && Heureactuelle < config.IntruFin)){
				// Nuit
				IntruD();
			}
			else{	// Jour
				IntruF();
			}
		}
		else{
			if(Heureactuelle > config.IntruDebut && Heureactuelle < config.IntruFin){
			 // Nuit
				IntruD();
			}
			else{	// Jour
				IntruF();
			}
		}
	Serial.print(F("Hintru = ")),Serial.print(Heureactuelle),Serial.print(",");
	Serial.print(config.IntruDebut),Serial.print(","),Serial.print(config.IntruFin);
	Serial.print(","),Serial.println(config.Intru);
}	
//---------------------------------------------------------------------------
void Buzzer(byte n){
	// n nombre de beep
	analogWriteFreq(4500);
	Alarm.delay(200);
	for(byte i=0;i<n;i++){
		analogWrite(OpBuzzer,512);//max1023
		Alarm.delay(100);
		analogWrite(OpBuzzer,0);//max1023
		Alarm.delay(20);
	}
}
//---------------------------------------------------------------------------
int Battpct(long vbat){
	//V2-15 calcul Etat batterie en %
	//V1-16 correction bug <= certaines lignes, remplacé par > partout
	int EtatBat = 0;
	if (vbat > 1260) {
		EtatBat = 100;
	}
	else if (vbat > 1255) {
		EtatBat = 95;
	}
	else if (vbat > 1250) {
		EtatBat = 90;
	}
	else if (vbat > 1246) {
		EtatBat = 85;
	}
	else if (vbat > 1242) {
		EtatBat = 80;
	}
	else if (vbat > 1237) {
		EtatBat = 75;
	}
	else if (vbat > 1232) {
		EtatBat = 70;
	}
	else if (vbat > 1226) {
		EtatBat = 65;
	}
	else if (vbat > 1220) {
		EtatBat = 60;
	}
	else if (vbat > 1213) {
		EtatBat = 55;
	}
	else if (vbat > 1206) {
		EtatBat = 50;
	}
	else if (vbat > 1198) {
		EtatBat = 45;
	}
	else if (vbat > 1190) {
		EtatBat = 40;
	}
	else if (vbat > 1183) {
		EtatBat = 35;
	}
	else if (vbat > 1175) {
		EtatBat = 30;
	}
	else if (vbat > 1167) {
		EtatBat = 25;
	}
	else if (vbat > 1158) {
		EtatBat = 20;		
	}
	else if (vbat > 1145) {
		EtatBat = 15;
	}
	else if (vbat > 1131) {
		EtatBat = 10;
	}
	else if (vbat > 1100) {
		EtatBat = 5;
	}
	else if (vbat <= 1050) {
		EtatBat = 0;
	}
	return EtatBat;
}
//---------------------------------------------------------------------------
void CreateJson(){

  // DynamicJsonDocument doc(600);
	// JsonObject JSONencoder = doc.createNestedObject("JSONencoder");
	// StaticJsonBuffer<600> JSONbuffer;
  // JsonObject& JSONencoder = JSONbuffer.createObject();
	
	// JSONencoder[F("Int")] 						= config.Intru;
	// JSONencoder[F("Silence")] 				= config.Silence;
	// JSONencoder[F("Vie")] 						= ChngFormatAlarme(config.Ala_Vie);
	// JSONencoder[F("Ds")] 							= config.Dsonn;
	// JSONencoder[F("DsMax")] 					= config.DsonnMax;
	// JSONencoder[F("Dsrepos")] 				= config.Dsonnrepos;
	// JSONencoder[F("J_TmCptMax")] 			= config.Jour_TmCptMax;
	// JSONencoder[F("J_Nmax")] 					= config.Jour_Nmax;
	// JSONencoder[F("IntA")] 						= config.IntruAuto;
	// JSONencoder[F("IntF")] 						= ChngFormatAlarme(config.IntruFin);
	// JSONencoder[F("IntD")] 						= ChngFormatAlarme(config.IntruDebut);
	// JSONencoder[F("AlaInt")]		 			= FlagAlarmeIntrusion;
	// JSONencoder[F("AlaV")] 						= FlagAlarmeTension;
	// JSONencoder[F("AlaSect")] 				= FlagAlarmeSect;
	// JSONencoder[F("VBatt")] 					= TensionBatterie;
	// JSONencoder[F("FsAla")] 					= FausseAlarme;
	// JSONencoder[F("CptAla")] 					= CptAla;
	// JSONencoder[F("ver")] 						= ver;
	// JSONencoder[F("Abs")] 						= config.Abs;
	// JSONencoder[F("Message")] 				= Reponse;
	// JSONencoder[F("Hsys")] 						= String(displayTime(1));
	
	String buffer = "";
	buffer = "{\"Int\":" + String(config.Intru) + ",";
	buffer += "\"Silence\":" + String(config.Silence) + ",";
	buffer += "\"Vie\":\"" + ChngFormatAlarme(config.Ala_Vie) + "\",";
	buffer += "\"Ds\":" + String(config.Dsonn) + ",";
	buffer += "\"DsMax\":" + String(config.DsonnMax) + ",";
	buffer += "\"Dsrepos\":" + String(config.Dsonnrepos) + ",";
	buffer += "\"J_TmCptMax\":" + String(config.Jour_TmCptMax) + ",";
	buffer += "\"J_Nmax\":" + String(config.Jour_Nmax) + ",";
	buffer += "\"IntA\":" + String(config.IntruAuto) + ",";
	buffer += "\"IntF\":\"" + ChngFormatAlarme(config.IntruFin) + "\",";
	buffer += "\"IntD\":\"" + ChngFormatAlarme(config.IntruDebut) + "\",";
	buffer += "\"AlaInt\":" + String(FlagAlarmeIntrusion) + ",";
	buffer += "\"AlaV\":" + String(FlagAlarmeTension) + ",";
	buffer += "\"AlaSect\":" + String(FlagAlarmeSect) + ",";
	buffer += "\"VBatt\":" + String(TensionBatterie) + ",";
	buffer += "\"FsAla\":" + String(FausseAlarme) + ",";
	buffer += "\"CptAla\":" + String(CptAlarme) + ",";
	buffer += "\"ver\":\"" + (ver) + "\",";
	buffer += "\"Abs\":" + String(config.Abs) + ",";
	buffer += "\"Message\":\"" + Reponse + "\",";
	buffer += "\"Hsys\":\"" + String(displayTime(1)+ "\"");
	buffer += "}";

	buffer.toCharArray(JSONmessageBuffer,(buffer.length() +1));
	// JSONencoder.prettyPrintTo(Serial);
	// JSONencoder.printTo(JSONmessageBuffer, sizeof(JSONmessageBuffer));
  Serial.println(JSONmessageBuffer);
	//Serial.print("envoi du buffer"), Serial.println(JSONmessageBuffer);
	// String localbuffer="";
	// serializeJson(doc,localbuffer);
}
//---------------------------------------------------------------------------
long Hms_long(String H){// convertir hh:mm:ss en long
	long Hnum = 0;
	// Debug.println(H);
	Hnum = H.substring(0,2).toInt();
	// Debug.println(Hnum);
	Hnum = Hnum*60;
	Hnum = Hnum + H.substring(3,5).toInt();
	// Debug.println(H.substring(3,5));
	// Debug.println(Hnum);
	Hnum = Hnum*60;
	Hnum = Hnum + H.substring(6,8).toInt();
	// Debug.println(H.substring(6,8));
	// Debug.println(Hnum);
	return Hnum;
}
//---------------------------------------------------------------------------
String ChngFormatAlarme(long mot){ // convertir heure long vers hh:mm:ss
	String sortie;
	int var = 0;
	var = int(mot / 3600);
	if (var < 10){
		sortie += "0";
		sortie += String(var);
	}
	else{
		sortie += String(var);
	}
	sortie += ":";
	var = int((mot % 3600)/60);
	if (var < 10){
		sortie += "0";
		sortie += String(var);
	}
	else{
		sortie += String(var);
	}
	sortie += ":";
	var = 60*((float(mot % 3600)/60) - int((mot % 3600)/60));
	if (var < 10){
		sortie += "0";
		sortie += String(var);
	}
	else{
		sortie += String(var);
	}
	return sortie;
}
//---------------------------------------------------------------------------
void MessageFaussesAlarmes(){
	if(FausseAlarme > 0){
		Reponse = F("Fausses Alarmes");
		CreateJson();
		rw_data(0);	// envoie du json sur bdd
		FausseAlarme = 0;
	}
}
//--------------------------------------------------------------------------------//
void configModeCallback (WiFiManager *myWiFiManager) {
  /* Called if WiFi has not been configured yet */
  Serial.println("Gestion Wifi"); // "Wifi Manager"
  Serial.println("Se connecter a "); // "Please connect to AP"
  Serial.println(myWiFiManager->getConfigPortalSSID());
  Serial.println("Acceder a la page de configuration"); // "To setup Wifi Configuration"
  Serial.println("192.168.4.1");
}
/* --------------------  commande serie ----------------------*/
void recvOneChar() {
  if (Serial.available() > 0) {
    receivedChar = Serial.read();
    MessageSerie += receivedChar;
    if (receivedChar == 10) {
      newData = true;
    }
  }
}

void showNewData() {
  if (newData == true) {
    Serial.println(MessageSerie);
    InterpreteEntree();
    newData = false;
  }
}
void InterpreteEntree() {
  String bidons;
  MessageSerie.toUpperCase();
  if (MessageSerie.indexOf("=") == 0) {
    bidons = MessageSerie.substring(1); //(MessageSerie.indexOf("=")+1);
    int lon0 = bidons.length();
    // bidons.toCharArray(replybuffer, lon0 - 1);	// len-1 pour supprimer lf
    // Serial.print(lon0),Serial.print(","),Serial.print(bidons),Serial.print(","),Serial.println(replybuffer);
    // traite_sms(99);	//	traitement SMS en mode test local
		Message = bidons;
		Serial.println(Message);
		InterpreteMessage(99);
  }

  MessageSerie = "";
}