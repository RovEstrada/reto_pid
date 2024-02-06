byte NTCPin = A0;
byte POTPin = A1;

#define SERIESRESISTOR 10000
#define NOMINAL_RESISTANCE 100000
#define NOMINAL_TEMPERATURE 25
#define BCOEFFICIENT 3950
#define NUMSAMPLES 10

const int TIP120Pin = 3; // Pin Digital PWM 3
const int SWPin = 4; // Pin Digital Lectura Potenciometro
uint16_t samples[NUMSAMPLES];

float k = 2.23; //ganancia
float theta = 0.2983; //Defasamiento
float tao = 0.7646; //Constante de tiempo
float T = 0.05; // 50 ms

// Valores de sintonizacion Ziegler-Nichols para PID
float kc = 1.2*tao/(k*theta);
float ti = 2*theta;
float td = theta/2;

// Coeficientes para la implementacion discreta del controlador PID
float q0 = kc*T/ti + kc + kc*td/T;
float q1 = -kc - 2*kc*td/T;
float q2 = kc*td/T;

float m[3]{}; // Arreglo con las tres manipulaciones mas recientes
float e[3]{}; // Arreglo con los tres errores mas recientes
int co = 0; // Controller Output

int t = 0; //Tiempo experimentacion
float temp = 0; // Temperature
float sp = 120; //Set-Point

int minutos = 0;
int segundos = 0;

// Limites de operacion para lazo cerrado
int lw = 20;
int up = 150;


float getTemperature(); // Metodo para obtener la temperatura recibida por el termistor
void desplazarDerecha(); // Metodo para actualizar los datos del arreglo de manipulacion y error
float sature(float, int); // Metodo que se asegura que la manipulacion que salga del controlador sea un valor entre 0 y 100%

void CLControl(); // Metodo que contiene el comportamiento del sistema de control de lazo cerrado (PID)
void OLControl(); //Metodo que contiene el comportamiento del sistema de control de lazo abierto
void printState(); // Metodo que se encarga de imprimir con formato variables sobre el estado del sistema y del controlador dependiendo 
                  // del modo de control actual (Lazo cerrado o abierto) 

void setup()
{
  pinMode(TIP120Pin, OUTPUT); // Declaracion como pin de salida asignado al componente TIP120Pin
  pinMode(SWPin, INPUT); // Declaraciion como pin de entrada asignado al Dip Switch
  Serial.begin(9600); // Inicializacion de comunicacion serial
  m[0] = 70; // Se inicia con una manipulacion de 70% por parte del controlador al arrancar el programa
}

void loop()
{
  unsigned long tiempoActual = millis(); // Obtiene el tiempo actual en milisegundos

  // Convierte milisegundos a minutos y segundos
  minutos = tiempoActual / 60000;
  segundos = (tiempoActual % 60000) / 1000;


  int op; // Variable que identifica el modo de operacion del controlador (ya sea lazo abierto o lazo cerrado)
  op = digitalRead(SWPin);

  if(op == 0)
    OLControl();
  else
    CLControl();
  
  printState(op);

  delay(50); // Wait for 50 millisecond(s)
}




float getTemperature(){
  float ADCvalue;
  float Resistance;

  float average;
 
  // take N samples in a row, with a slight delay
  for (int i=0; i< NUMSAMPLES; i++) {
   samples[i] = analogRead(NTCPin);
   delay(10);
  }
 
  // average all the samples out
  average = 0;
  for (int i=0; i< NUMSAMPLES; i++) {
     average += samples[i];
  }
  average /= NUMSAMPLES;

  ADCvalue = average;

  //convert value to resistance
  Resistance = (1023 / ADCvalue) - 1;
  Resistance = SERIESRESISTOR / Resistance;

  float steinhart;
  steinhart = Resistance / NOMINAL_RESISTANCE; // (R/Ro)
  steinhart = log(steinhart); // ln(R/Ro)
  steinhart /= BCOEFFICIENT; // 1/B * ln(R/Ro)
  steinhart += 1.0 / (NOMINAL_TEMPERATURE + 273.15); // + (1/To)
  steinhart = 1.0 / steinhart; // Invert
  steinhart -= 273.15; // convert to C
  // Serial.print(ADCvalue);
  // Serial.print(",");
  // Serial.print(steinhart);
  return steinhart;
}

void desplazarDerecha(float *array) {
    // array[1] = array[0];
    // array[2] = array[1];
    array[2] = array[1];
    array[1] = array[0];
}

float sature(float m, int up){
  if(m < 0)
    return 0;
  else
    return min(m, up); //El impulso maximo sera 255 
}

void OLControl(){
  desplazarDerecha(m);  
  temp = getTemperature();
  long p = analogRead(POTPin); // Lectura de potenciometro

  m[0] = 100*p/1023; //Calculo de manipulacion en porcentaje

  co = m[0]*2.55;  // Escala de valor para obtener un valor PWM de entre 0-255
  
  analogWrite(TIP120Pin, co); // Pulso PWM para el actuador
}

void CLControl(){
  long p = analogRead(POTPin); //Lectura de potenciometro para obtener un SP
  sp = p*(up-lw)/1023 + lw; // Obtencion de SP entre [20,150] en C
  temp = getTemperature();  // Obtencion de temperatura del sistema
  desplazarDerecha(e); // Actualizacion de valores en arreglo de errores
  desplazarDerecha(m);  // Actualizacion de valores en arreglo de manipulaciones

  e[0] = sp - temp; // Calculo de error actual
  m[0] = m[1] + q0*e[0] + q1*e[1] + + q2*e[2]; // Calculo de manipulacion actual
  
  m[0] = sature(m[0], 100); // Saturacion de manipulacion calculada limitada a un rango de [0,100] en porcentaje
  co = 2.55*m[0]; // Escalamiento a valor de entre [0,255] de PMW 
  
  analogWrite(TIP120Pin, co); // Mandamos el valor PWM de manipulacion al actuador
}

void printState(int op){
    // Imprime el tiempo en el formato MM:SS
  Serial.print("t:");
  Serial.print(minutos);
  Serial.print(':');
  if (segundos < 10) {
    Serial.print('0'); // Añade un cero al frente si los segundos son menores a 10
  }
  Serial.print(segundos);

  if(op == 0){
    Serial.print(", temp:");
    Serial.print(temp);
    Serial.print(", CO:");
    Serial.print(m[0]);
    Serial.print(", PWM:");
    Serial.println(co);
  }
  else{
    Serial.print(", setpoint:");
    Serial.print(sp);
    Serial.print(", temp:");
    Serial.print(temp);
    Serial.print(", error:");
    Serial.print(e[0]);
    Serial.print(", outpid:");
    Serial.print(m[0]);
    Serial.print(", outpidPWM:");
    Serial.println(co);
  }

  
}