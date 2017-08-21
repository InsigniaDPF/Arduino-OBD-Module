#include <SPI.h>
#include "mcp_can.h"

const int SPI_CS_PIN = 9;

MCP_CAN CAN(SPI_CS_PIN);

#define FILTER0 0x7DF   //Filtr 0 dla ramek nadawanych z adresu 0x7DF
#define FILTER1 0x7E0   //Filtr 1 dla ramek nadawanych z adresów 0x7E0 oraz 0x7E1

#define MASK0   0x7FF   //Maska 0 dla ramek nadawanych z adresu 0x7DF
#define MASK1   0x7FE   //Mask 1 dla ramek nadawanych z adresó 0x7E0 oraz 0x7E1
#define MODULE_ADDRESS0  0x7E9  //Adres CAN modułu z zakresu 0x7Ex
#define MODULE_ADDRESS1  0x5E9  //Adres CAN modułu z zakresu 0x5Ex
#define OBD_MODE_01_PID_A 0x88  //Pod tym PIDem wyślemy odpowiedź
#define OBD_MODE_01_PID_B 0x0C  //PID odczytu RPM, ale my też będziemy na niego odpowiadać
#define OBD_MODE_22_PID 0x000C  //PID odczytu RPM w trybie 22
#define OBD_MODE_AA_TABLE 0xFE  //Tablica do której również wyśl
#define OBD_MODE_AA_RESPONCE_TABLE 0x01


#define VOLT_REF 5.0

#define DEBUG 1
#define DEBUG_SENSOR
#define SERIALBAUD 115200    //Prędkość transmisji na porcie COM w trybie DEBUG

unsigned char flagRecv = 0;   //Flaga ustawiana gdy MCP2151 zgłosi odbiór ramki CAN
unsigned char can_rx_len = 0; //Długość odebranej ramki CAN
unsigned char can_rx_buff[8]; //Bufor odbiorczy danych ramki CAN

unsigned char can_tx_len = 8; //Długość nadawanej ramki CAN, zawsze wysyłamy 8 bajtów danych
unsigned char can_tx_buff[8]; //Bufoe nadawczy danych ramki CAN

unsigned int obd_oil_press_val = 0;

void setup() {

        #ifdef DEBUG
          Serial.begin(SERIALBAUD);
          Serial.println("START....");
          Serial.println("Config MCP2515");
        #endif


  //Konfiguracja MCP2515
        while (CAN_OK != CAN.begin(CAN_500KBPS))    // init can bus : baudrate = 500k
        {
              #ifdef DEBUG
                  Serial.println("BŁĄD Konfiguracji, sprawdz urzadzenie");
              #endif
          delay(100);
        }
        #ifdef DEBUG
          Serial.println("MCP2515 skonfigurowany dla prędkości CAN_500KBPS");
        #endif


        /*
           Ustawienie masek dla ramek przychodzących, do dyspozycji mamy 2 maski
        */
      #ifdef DEBUG
        Serial.println("Konfiguracja Maski i Filtru 1");
      #endif
        CAN.init_Mask(0, 0, MASK0);
        CAN.init_Filt(0, 0, FILTER0);
        CAN.init_Filt(1, 0, FILTER0);

      #ifdef DEBUG
        Serial.println("Konfiguracja Maski i Filtru 2");
      #endif
        CAN.init_Mask(1, 0, MASK1);

        /*
           Ustawienie filtrów dla ramek przychodzących , do dyspozycji mamy 5 filtrów.
        */
      
        CAN.init_Filt(2, 0, FILTER1);
        CAN.init_Filt(3, 0, FILTER1);
        CAN.init_Filt(4, 0, FILTER1);
        CAN.init_Filt(5, 0, FILTER1);

      #ifdef DEBUG
        Serial.println("KONFIGURACJA PRZERWAŃ");
      #endif
      /* Ustawienie przerwania na pinie INT0 od opadajacego zbocza */  
      attachInterrupt(0, MCP2515_ISR, FALLING); // start interrupt



      #ifdef DEBUG
        Serial.println("URZĄDZENIE GOTOWE DO PRACY");
        Serial.println("OCZEKIWANIE NA KOMENDY OBD");
      #endif

}

void loop() {
  

  if(flagRecv)    //Jeżeli odebrana została ramka CAN (flagRecv różne od 0)
  {
    flagRecv = 0;
      #ifdef DEBUG
          Serial.print("PARAMETR CZUJNIKA OLEJU\tHEX: ");
          Serial.print(obd_oil_press_val, HEX);
          Serial.print("\t\tNAPIECIE: ");
          float napiecie = obd_oil_press_val * VOLT_REF / 1023;
          Serial.print(napiecie);  //1023 dlatego że przetwornik jest 10-cio bitowy, co daje 1 bit na każde ~0,005V
          Serial.print("V\tCISNIENIE: ");
          float bar = -12.4 * napiecie + 11.98;
          Serial.print(bar);
          Serial.print(" bar\t");
      
          Serial.print("\t-->MCP2515:");
      #endif
     

      //Odczytuj ramki z układu MCP2515, Układ posiada 2 bufory odbiorcze, dlatego trzeba sprawdzić wszystkie z nich.
      while (CAN_MSGAVAIL == CAN.checkReceive()) //Pobierz ramki CAN z bufora
      {
        
        CAN.readMsgBuf(&can_rx_len, can_rx_buff);
        
        #ifdef DEBUG
          
          Serial.print("\t\tRAMKA CAN: ");
          Serial.print(CAN.getCanId(), HEX);
          for(int i=0;i<can_rx_len; i++)
          {
            Serial.print(" ");
            Serial.print(can_rx_buff[i], HEX);
          }
        
        #endif
        
        if(can_rx_buff[0]<0x08 )            //Tylko w przypadku gdy wiadomość CAN jest jednoramkowa (długość wiadomości mniejsza niż 8 bajtów)
          //Dla trybu 0x01
          if(can_rx_buff[1] == 0x01 && (can_rx_buff[2] == OBD_MODE_01_PID_A || can_rx_buff[2] == OBD_MODE_01_PID_B)) //Tlko w przypadku gdy wiadomość jest Trybu 1, oraz PID to 88 (nieokreslone dla GM), lub PID to 0C (RPM)
          {
              #ifdef DEBUG
                Serial.print("\tMODE 01, PID: ");
                Serial.print(can_rx_buff[2], HEX);
              #endif

              //Wysyłanie odpowiedzi
              can_tx_buff[0] = 0x04;                      //Długość wiadomości 4 Bajty
              can_tx_buff[1] = 0x41;                      //Odpowiedź na MODE_01 (0x01 + 0x40)
              can_tx_buff[2] = can_rx_buff[2];            //Wzywany PID
              can_tx_buff[3] = obd_oil_press_val >> 8;    //Starszy Bajt ciśnienia Oleju
              can_tx_buff[4] = obd_oil_press_val & 0xFF;  //Młodszy Bajt ciśnienia Oleju
              can_tx_len = 5;
  
              CAN.sendMsgBuf(MODULE_ADDRESS0, 0, can_tx_len, can_tx_buff); //Wysyłanie ramki na CAN
  
              #ifdef DEBUG
                Serial.print("\tODPOWIEZD: ADRES ");
                Serial.print(MODULE_ADDRESS0, HEX);
                for(int i=0;i<can_tx_len; i++)
                {
                  Serial.print(" ");
                  Serial.print(can_tx_buff[i], HEX);
                }
                Serial.println();
              #endif
          }
          //Dla trybu 22
          else if(can_rx_buff[1] == 0x22 && ((can_rx_buff[2]<<8 | can_rx_buff[3]) == OBD_MODE_01_PID_A) || ((can_rx_buff[2]<<8 | can_rx_buff[3]) == OBD_MODE_01_PID_B))//Podobnie jak dla Trybu 1, z tym że PID zapisany jest 2 dwóch bajtach
          {
              #ifdef DEBUG
                Serial.print("\tMODE 22, PID: 00");
                Serial.print(can_rx_buff[2]<<8 | can_rx_buff[3], HEX);
              #endif

              can_tx_buff[0] = 0x05;                    //Długość 5 Bajtów
              can_tx_buff[1] = 0x62;                    //Odpowiedź na MODE_22 (0x22 + 0x40)
              can_tx_buff[2] = can_rx_buff[2];          //Wzywany PID, starszy bajt
              can_tx_buff[3] = can_rx_buff[3];          //Wzywany PID, młodszy bajt
              can_tx_buff[4] = obd_oil_press_val >> 8;    //Starszy Bajt ciśnienia Oleju
              can_tx_buff[5] = obd_oil_press_val & 0xFF;   //Młodszy Bajt ciśnienia Oleju
              can_tx_len = 6;

              CAN.sendMsgBuf(MODULE_ADDRESS0, 0, can_tx_len, can_tx_buff); //Wysyłanie ramki na CAN

              #ifdef DEBUG
                Serial.print("\tODPOWIEDZ\tADRES");
                Serial.print(MODULE_ADDRESS0, HEX);
                for(int i=0;i<can_tx_len; i++)
                {
                  Serial.print(" ");
                  Serial.print(can_tx_buff[i], HEX);
                }
                Serial.println();
              #endif
          }
          //Dla trybu 0xAA
          else if(can_rx_buff[1] == 0xAA && can_rx_buff[2] == 0x01 &&                             //Tryb AA jest inaczej obsługiwany, w projekcie przyjęto założenie, że moduł będzie reagował tylko wtedy podtryb to 0x01, a wzywana tablica to 0xFE                  
          ( can_rx_buff[3] == OBD_MODE_AA_TABLE || can_rx_buff[4] == OBD_MODE_AA_TABLE ||
            can_rx_buff[5] == OBD_MODE_AA_TABLE || can_rx_buff[6] == OBD_MODE_AA_TABLE || 
            can_rx_buff[7] == OBD_MODE_AA_TABLE)) 
          {
              #ifdef DEBUG
                Serial.print("\tMODE AA, TABLE: ");
                Serial.print(OBD_MODE_AA_TABLE, HEX);
              #endif

              can_tx_buff[0] = OBD_MODE_AA_RESPONCE_TABLE;                //Numer wzywanej tablicy, moduł nie może korystać z tablicy która została zdefiniowana w trybie AA gdyż została zaalokowana do ECU, dlatego moduł uzyje tablicy 01
              can_tx_buff[1] = obd_oil_press_val >> 8;                    //Starszy Bajt ciśnienia Oleju
              can_tx_buff[2] = obd_oil_press_val & 0xFF;                  //Młodszy Bajt ciśnienia Oleju
              can_tx_len = 3;

              CAN.sendMsgBuf(MODULE_ADDRESS1, 0, can_tx_len, can_tx_buff); //Wysyłanie ramki na CAN
              
              #ifdef DEBUG
                Serial.print("\tODPOWIEDZ\tADRES ");
                Serial.print(MODULE_ADDRESS1, HEX);
                for(int i=0;i<can_tx_len; i++)
                {
                  Serial.print(" ");
                  Serial.print(can_tx_buff[i], HEX);
                }
                Serial.println();
              #endif
          }
          #ifdef DEBUG
          else
          {
            #ifdef DEBUG
                Serial.print("\tMODE ");
                Serial.print(can_rx_buff[1], HEX);
                Serial.println("\tNIE WYSYLAM ODPOWIEDZI");
              #endif
          }
          #endif
              obd_oil_press_val = analogRead(A5);
          #ifdef DEBUG_SENSOR
            Serial.print("PARAMETR CZUJNIKA OLEJU\tHEX: ");
            Serial.print(obd_oil_press_val, HEX);
            Serial.print("\t\tNAPIECIE: ");
            float napiecie = obd_oil_press_val * VOLT_REF / 1023;
            Serial.print(napiecie);  //1023 dlatego że przetwornik jest 10-cio bitowy, co daje 1 bit na każde ~0,005V
            Serial.print("V\tCISNIENIE: ");
            float bar = -12.4 * napiecie + 11.98;
            Serial.print(bar);
            Serial.println(" bar");
            
          #endif
      
      }

  }

  obd_oil_press_val = analogRead(A5);
  #ifdef DEBUG_SENSOR
    Serial.print("PARAMETR CZUJNIKA OLEJU\tHEX: ");
    Serial.print(obd_oil_press_val, HEX);
    Serial.print("\tNAPIECIE: ");
    float napiecie = obd_oil_press_val * VOLT_REF / 1023;
    Serial.print(napiecie);  //1023 dlatego że przetwornik jest 10-cio bitowy, co daje 1 bit na każde ~0,005V
    Serial.print("V\tCISNIENIE: ");
    float bar = -12.4 * napiecie + 11.98;
    Serial.print(bar);
    Serial.println(" bar");
    
  #endif
  
}

void MCP2515_ISR()
{
    flagRecv = 1;
}
