#include <SimpleTimer.h>
#include <RPLidar.h> // This sketch code is based on the RPLIDAR driver library provided by RoboPeak

SimpleTimer timer; // Timer pour échantillonnage
RPLidar lidar; // You need to create an driver instance

#define RPLIDAR_MOTOR 3 // The PWM pin for control the speed of RPLIDAR's motor.
                        // This pin should connected with the RPLIDAR's MOTOCTRL signal

// unsigned int tick_codeuse = 0; // Compteur de tick de la codeuse
int vitMoteur = 0; // Commande du moteur
const int frequence_echantillonnage = 100; // Fréquence d'exécution de l'asservissement

// Rapport entre le nombre de tours de l'arbre moteur et de la roue
// (電機軸與葉輪的轉數比)
const int rapport_reducteur = 90;

const int tick_par_tour_codeuse = 16; // 64 tick sur deux capteurs hall, ici on a pris un seul capteur

// max = 3 tours/sec soit 180 tours/min
// (最大 = 3 轉/秒，即 180 轉/分鐘)
float RPS2PWM = 255/3;

// consigne en tour/s; consigne nombre de tours de roue par seconde
// (以轉/秒為單位的設定值); (設置車輪每秒轉數)
float consigne_moteur = 0.5;

// init calculs asservissement PID
float erreur_precedente = consigne_moteur; // (en tour/s)
float somme_erreur = 0;

// Definition des constantes du correcteur PID
float kp = 200; // Coefficient proportionnel; choisis par tatonnement sur le moniteur. Ce sont les valeurs qui donnaient les meilleures performances
float ki = 2; // 5.5; Coefficient intégrateur
float kd = 0; // Coefficient dérivateur

// Arduino PWM Speed Control：
const int E1 = 5;
int M1 = 12;
const int E2 = 6;
int M2 = 13;

// The sample code for driving one way motor encoder 1
const byte encoder1pinA = 18; // A pin -> the interrupt pin 18 (int 5)
const byte encoder1pinB = 9; // B pin -> the digital pin 9
byte encoder1PinALast = LOW;
long duration1 = 0; // the number of the pulses
boolean Direction1; // the rotation direction

// The sample code for driving one way motor encoder 2
const byte encoder2pinA = 19; // A pin -> the interrupt pin 19 (int 4)
const byte encoder2pinB = 10; // B pin -> the digital pin 10
byte encoder2PinALast = LOW;
long duration2 = 0; // the number of the pulses
boolean Direction2; // the rotation direction

float perimetre = 32.0442; // 2*5.1*pi
float distance = 0;
float angle = 0;
int tmr_id[4]; // bouger
int id_c = 0; // sign de bouger
int etape_change = 0;

float minDistance = 100000;
float angleAtMinDist = 0;
byte flag = 0;

void EncoderInit() {
  Direction1 = true; // default -> Forward
  Direction2 = false;
  pinMode(encoder1pinB, INPUT);
  pinMode(encoder2pinB, INPUT);
  attachInterrupt(5, wheelSpeed1, CHANGE); // int.5
  attachInterrupt(4, wheelSpeed2, CHANGE); // int.4
}

// Read quadrature encoder1
void wheelSpeed1() {
  int Lstate1 = digitalRead(encoder1pinA);
  if((encoder1PinALast == LOW) && Lstate1 == HIGH) {
    int val1 = digitalRead(encoder1pinB);
    if(val1 == LOW && Direction1) {
      Direction1 = false; // Reverse
    } else if(val1 == HIGH && !Direction1) {
      Direction1 = true; // Forward
    }
  }
  encoder1PinALast = Lstate1;
  if(!Direction1) {
    duration1++;
  } else {
    duration1--;
  }
}

// Read quadrature encoder2
void wheelSpeed2() { 
  int Lstate2 = digitalRead(encoder2pinA);
  if((encoder2PinALast == LOW) && Lstate2 == HIGH) {
    int val2 = digitalRead(encoder2pinB);
    if(val2 == LOW && Direction2) {
      Direction2 = false; // Reverse
    } else if(val2 == HIGH && !Direction2) {
      Direction2 = true; // Forward
    }
  }
  encoder2PinALast = Lstate2;
  if(!Direction2) {
    duration2--;
  } else {
    duration2++;
  }
}

/* Interruption pour calcul du P */
void asservissement_av() {
  // Calcul de l'erreur
  int frequence_codeuse = frequence_echantillonnage*(duration1+duration2)/2; // 100*tick_codeuse
  float vit_roue_tour_sec = (float)frequence_codeuse/(float)tick_par_tour_codeuse/(float)rapport_reducteur; // (100*tick_codeuse)/32/19 
  float erreur = consigne_moteur - vit_roue_tour_sec; // pour le proportionnel
  somme_erreur += erreur; // pour l'intégrateur
  float delta_erreur = erreur - erreur_precedente;  // pour le dérivateur
  erreur_precedente = erreur;
  
  // Réinitialisation du nombre de tick de la codeuse
  // (編碼器滴答計數重置)
  duration1 = 0;
  duration2 = 0;

  // P : calcul de la commande
  vitMoteur = kp*erreur + ki*somme_erreur + kd*delta_erreur;  // somme des tois erreurs

  // Normalisation et contrôle du moteur
  if (vitMoteur > 255) {
    vitMoteur = 255;  // sachant que l'on est branché sur un pont en H L293D
  } else if (vitMoteur < 0) {
    vitMoteur = 0;
  }

  Avance(vitMoteur);
  distance += vit_roue_tour_sec*perimetre/(rapport_reducteur*2);
  
  // l'erreur = 8
  /*Serial.print(vit_roue_tour_sec); // la vitesse
  Serial.print(", ");
  Serial.print(consigne_moteur*RPS2PWM); // deg/sec
  Serial.print(", ");
  Serial.print(vitMoteur); // pwm
  Serial.print(", ");
  Serial.print(distance);
  Serial.println();*/
}

/* Interruption pour calcul du P */
void asservissement_dr() {
  // Calcul de l'erreur
  int frequence_codeuse = frequence_echantillonnage*(duration1+(-duration2))/2; // 100*tick_codeuse
  float vit_roue_tour_sec = (float)frequence_codeuse/(float)tick_par_tour_codeuse/(float)rapport_reducteur; // (100*tick_codeuse)/32/19 
  float erreur = consigne_moteur - vit_roue_tour_sec; // pour le proportionnel
  somme_erreur += erreur; // pour l'intégrateur
  float delta_erreur = erreur - erreur_precedente;  // pour le dérivateur
  erreur_precedente = erreur;
  
  // Réinitialisation du nombre de tick de la codeuse
  // (編碼器滴答計數重置)
  duration1 = 0;
  duration2 = 0;

  // P : calcul de la commande
  vitMoteur = kp*erreur + ki*somme_erreur + kd*delta_erreur;  // somme des tois erreurs

  // Normalisation et contrôle du moteur
  if (vitMoteur > 255) {
    vitMoteur = 255;  // sachant que l'on est branché sur un pont en H L293D
  } else if (vitMoteur < 0) {
    vitMoteur = 0;
  }

  Droit(vitMoteur);
  
  distance += vit_roue_tour_sec*perimetre/(rapport_reducteur*2);
  angle = distance/perimetre*360/2;
  
  // l'erreur = 8
  /*Serial.print(vit_roue_tour_sec); // la vitesse
  Serial.print(", ");
  Serial.print(consigne_moteur*RPS2PWM); // deg/sec
  Serial.print(", ");
  Serial.print(vitMoteur); // pwm
  Serial.print(", ");
  Serial.print(angle);
  Serial.println();*/
}

/* Interruption pour calcul du P */
void asservissement_ga() {
  // Calcul de l'erreur
  int frequence_codeuse = frequence_echantillonnage*(duration1+(-duration2))/2; // 100*tick_codeuse
  float vit_roue_tour_sec = (float)frequence_codeuse/(float)tick_par_tour_codeuse/(float)rapport_reducteur; // (100*tick_codeuse)/32/19 
  float erreur = consigne_moteur - vit_roue_tour_sec; // pour le proportionnel
  somme_erreur += erreur; // pour l'intégrateur
  float delta_erreur = erreur - erreur_precedente;  // pour le dérivateur
  erreur_precedente = erreur;
  
  // Réinitialisation du nombre de tick de la codeuse
  // (編碼器滴答計數重置)
  duration1 = 0;
  duration2 = 0;

  // P : calcul de la commande
  vitMoteur = kp*erreur + ki*somme_erreur + kd*delta_erreur;  // somme des tois erreurs

  // Normalisation et contrôle du moteur
  if (vitMoteur > 255) {
    vitMoteur = 255;  // sachant que l'on est branché sur un pont en H L293D
  } else if (vitMoteur < 0) {
    vitMoteur = 0;
  }

  Gauche(vitMoteur);
  
  distance += vit_roue_tour_sec*perimetre/(rapport_reducteur*2);
  angle = distance/perimetre*360/2;
  
  // l'erreur = 8
  /*Serial.print(vit_roue_tour_sec); // la vitesse
  Serial.print(", ");
  Serial.print(consigne_moteur*RPS2PWM); // deg/sec
  Serial.print(", ");
  Serial.print(vitMoteur); // pwm
  Serial.print(", ");
  Serial.print(angle);
  Serial.println();*/
}

/* Interruption pour calcul du P */
void asservissement_re() {
  // Calcul de l'erreur
  int frequence_codeuse = frequence_echantillonnage*(duration1+duration2)/2; // 100*tick_codeuse
  float vit_roue_tour_sec = (float)frequence_codeuse/(float)tick_par_tour_codeuse/(float)rapport_reducteur; // (100*tick_codeuse)/32/19
  float erreur = consigne_moteur - vit_roue_tour_sec; // pour le proportionnel
  somme_erreur += erreur; // pour l'intégrateur
  float delta_erreur = erreur - erreur_precedente;  // pour le dérivateur
  erreur_precedente = erreur;
  
  // Réinitialisation du nombre de tick de la codeuse
  // (編碼器滴答計數重置)
  duration1 = 0;
  duration2 = 0;

  // P : calcul de la commande
  vitMoteur = kp*erreur + ki*somme_erreur + kd*delta_erreur;  // somme des tois erreurs

  // Normalisation et contrôle du moteur
  if (vitMoteur > 255) {
    vitMoteur = 255;  // sachant que l'on est branché sur un pont en H L293D
  } else if (vitMoteur < 0) {
    vitMoteur = 0;
  }

  Recule(vitMoteur);
  
  distance += vit_roue_tour_sec*perimetre/(rapport_reducteur*2);
  
  // l'erreur = 8
  /*Serial.print(vit_roue_tour_sec); // la vitesse
  Serial.print(", ");
  Serial.print(consigne_moteur*RPS2PWM); // deg/sec
  Serial.print(", ");
  Serial.print(vitMoteur); // pwm
  Serial.print(", ");
  Serial.print(distance);
  Serial.println();*/
}

void Arret() {
  digitalWrite(M1, LOW);
  digitalWrite(M2, LOW);
  analogWrite(E1, LOW);
  analogWrite(E2, LOW);
  distance = 0;
  angle = 0;
}

void Avance(int powerRate) {
  digitalWrite(M1, LOW);
  digitalWrite(M2, LOW);
  analogWrite(E1, powerRate);
  analogWrite(E2, powerRate);
}

void Recule(int powerRate) {
  digitalWrite(M1, HIGH);
  digitalWrite(M2, HIGH);
  analogWrite(E1, powerRate);
  analogWrite(E2, powerRate);
}

void Droit(int powerRate) {
  digitalWrite(M1, HIGH);
  digitalWrite(M2, LOW);
  analogWrite(E1, powerRate);
  analogWrite(E2, powerRate);
}

void Gauche(int powerRate) {
  digitalWrite(M1, LOW);
  digitalWrite(M2, HIGH);
  analogWrite(E1, powerRate);
  analogWrite(E2, powerRate);
}

void Avance_control(int dis) {
  timer.enable(tmr_id[0]);
  timer.disable(tmr_id[1]);
  timer.disable(tmr_id[2]);
  timer.disable(tmr_id[3]);
  if(distance > dis) {
    timer.disable(tmr_id[0]);
    Arret();
    etape_change++;
  }
}

void Recule_control(int dis) {
  timer.disable(tmr_id[0]);
  timer.disable(tmr_id[1]);
  timer.disable(tmr_id[2]);
  timer.enable(tmr_id[3]);
  if(distance < -dis) {
    timer.disable(tmr_id[3]);
    Arret();
    etape_change++;
  }
}

void Droit_control(int ang) {
  timer.disable(tmr_id[0]);
  timer.enable(tmr_id[1]);
  timer.disable(tmr_id[2]);
  timer.disable(tmr_id[3]);
  if(angle < -ang) {
    timer.disable(tmr_id[1]);
    Arret();
    etape_change++;
  }
}

void Gauche_control(int ang) {
  timer.disable(tmr_id[0]);
  timer.disable(tmr_id[1]);
  timer.enable(tmr_id[2]);
  timer.disable(tmr_id[3]);
  if(angle > ang) {
    timer.disable(tmr_id[2]);
    Arret();
    etape_change++;
  }
}

void _display(float angle, float distance, char div) {
  bool _dist = (distance > 550.0) ? false : true;
  if(_dist) {
    if(div == '4') {
      if((angle < 45.0 && angle >= 0.0) || (angle < 359.0 && angle >= 315.0)) {
        // Serial.println("1"); // Avant
        flag = 1;
      } else if(angle < 135.0 && angle >= 45.0) {
        // Serial.println("2"); // droit
        flag = 2;
      } else if(angle < 225.0 && angle >= 135.0) {
        // Serial.println("3"); // arriere
        flag = 3;
      } else if(angle < 315.0 && angle >= 225.0) {
        // Serial.println("4"); // gauche
        flag = 4;
      } else {
        flag = 0;
      }
    }
    if(div == '6') {
      if((angle < 30.0 && angle >= 0.0) || (angle < 359.0 && angle >= 330.0)) {
        // Serial.println("1");
        flag = 1;
      } else if(angle < 90.0 && angle >= 30.0) {
        // Serial.println("2");
        flag = 2;
      } else if(angle < 150.0 && angle >= 90.0) {
        // Serial.println("3");
        flag = 3;
      } else if(angle < 210.0 && angle >= 150.0) {
        // Serial.println("4");
        flag = 4;
      } else if(angle < 270.0 && angle >= 210.0) {
        // Serial.println("5");
        flag = 5;
      } else if(angle < 330.0 && angle >= 270.0) {
        // Serial.println("6");
        flag = 6;
      } else {
        flag = 0;
      }
    }
    if(div == '8') {
      if((angle < 22.5 && angle >= 0.0) || (angle < 359.0 && angle >= 337.5)) {
        // Serial.println("1");
        flag = 1;
      } else if(angle < 67.5 && angle >= 22.5) {
        // Serial.println("2");
        flag = 2;
      } else if(angle < 112.5 && angle >= 67.5) {
        // Serial.println("3");
        flag = 3;
      } else if(angle < 157.5 && angle >= 112.5) {
        // Serial.println("4");
        flag = 4;
      } else if(angle < 202.5 && angle >= 157.5) {
        // Serial.println("5");
        flag = 5;
      } else if(angle < 247.5 && angle >= 202.5) {
        // Serial.println("6");
        flag = 6;
      } else if(angle < 292.5 && angle >= 247.5) {
        // Serial.println("7");
        flag = 7;
      } else if(angle < 337.5 && angle >= 292.5) {
        // Serial.println("8");
        flag = 8;
      } else {
        flag = 0;
      }
    }
  } else {
    flag = 0;
  }
}

void rpLidar() {
  if (IS_OK(lidar.waitPoint())) {
    float _distance = lidar.getCurrentPoint().distance; // distance value in mm unit
    float _angle    = lidar.getCurrentPoint().angle; // anglue value in degree
    bool  _startBit = lidar.getCurrentPoint().startBit; // whether this point is belong to a new scan
    byte  _quality  = lidar.getCurrentPoint().quality; // quality of the current measurement
    // perform data processing here...
    /*Serial.print("current angle : ");
    Serial.println(angle, DEC);
    Serial.print("current distance : ");
    Serial.println(distance, DEC);*/
    // a new scan, display previous data...
    if(_startBit) {
      _display(angleAtMinDist, minDistance, '4');
      minDistance = 100000;
      angleAtMinDist = 0;
    } else {
      if(_distance > 0 && _distance < minDistance) {
        minDistance = _distance;
        angleAtMinDist = _angle;
      }
    }
  } else {
    analogWrite(RPLIDAR_MOTOR, 0); //stop the rplidar motor
    // try to detect RPLIDAR... 
    rplidar_response_device_info_t info;
    if (IS_OK(lidar.getDeviceInfo(info, 100))) {
       // detected...
       lidar.startScan();
       // start motor rotating at max allowed speed
       analogWrite(RPLIDAR_MOTOR, 255);
       delay(1000);
    }
  }
}

/* Routine d'initialisation */
void setup() {
  // Serial.begin(57600); // Initialisation port COM
  pinMode(E1, OUTPUT); // Sorties commande moteur 1
  pinMode(E2, OUTPUT); // Sorties commande moteur 2
  pinMode(M1, OUTPUT);
  pinMode(M2, OUTPUT);

  analogWrite(E1, 0); // Initialisation sortie moteur à 0 
  analogWrite(E2, 0);
  // Pause de 0,3 sec pour laisser le temps au moteur de s'arréter si celui-ci est en marche
  // (暫停 0.3 秒，讓電機在運行時有時間停止)
  delay(300);

  EncoderInit(); // Initialize the module

  tmr_id[0] = timer.setInterval(1000/frequence_echantillonnage, asservissement_av); // +
  tmr_id[1] = timer.setInterval(1000/frequence_echantillonnage, asservissement_dr); // -
  tmr_id[2] = timer.setInterval(1000/frequence_echantillonnage, asservissement_ga); // +
  tmr_id[3] = timer.setInterval(1000/frequence_echantillonnage, asservissement_re); // -
  
  lidar.begin(Serial2); // bind the RPLIDAR driver to the arduino hardware serial
  pinMode(RPLIDAR_MOTOR, OUTPUT); // set pin modes
}

/* Fonction principale */
void loop(){
  timer.run(); // on fait tourner l'horloge (我們運行時鐘)
  /*switch(etape_change) {
    case 0:
      Avance_control(30);
      break;
    case 1:
      Gauche_control(90);
      break;
    case 2:
      Avance_control(30);
      break;
    case 3:
      Droit_control(90);
      break;
    case 4:
      Avance_control(30);
      break;
    case 5:
      Gauche_control(90);
      break;
    case 6:
      Avance_control(30);
      break;
    case 7:
      Droit_control(90);
      break;
  }
  if(etape_change >= 8) {
    etape_change = 0;
  }*/
  rpLidar();
  /*switch(flag) {
    case 0:
      Arret();
      break;
    case 1:
      Avance_control(30);
      break;
    case 2:
      Droit_control(90);
      break;
    case 3:
      Recule_control(30);
      break;
    case 4:
      Gauche_control(90);
      break;
  }*/
  switch(flag) {
    case 0:
      Arret();
      break;
    case 1:
      Recule_control(40);
      break;
    case 2:
      Gauche_control(90);
      break;
    case 3:
      Avance_control(40);
      break;
    case 4:
      Droit_control(90);
      break;
  }
  /*switch(flag) {
    case 0:
      Arret();
      break;
    case 1:
      Avance_control(30);
      break;
    case 2:
      Gauche_control(45);
      break;
    case 3:
      Avance_control(30);
      break;
    case 4:
      Droit_control(45);
      break;
    case 5:
      Avance_control(30);
      break;
    case 6:
      Droit_control(45);
      break;
    case 7:
      Avance_control(30);
      break;
    case 8:
      Gauche_control(45);
      break;
  }*/
}
