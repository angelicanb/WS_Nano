
/**********************************************
 * Definição dos pinos                        *
 **********************************************/
//Motor
#define VELOCIDADE_in     3   // A2  // Leitura da velocidade do motor
#define PWM_out           5   // Escrita da velocidade do motor
#define DIRECAO           7   // Escrita da direção do motor

//Leituras
#define LED_DADOS         13    // 
#define FIM_CURSO         2     // Leitura do sensor 
#define LEIT_PWR_IN       A4    // Leitura da tensão da bateria;
#define LEIT_CORRENTE     A0    // Leitura da corrente do motor.

/**********************************************
 * Definição das constantes                   *
 **********************************************/
#define BAUD_RATE                 9600
#define PWM_MAX                   255 
#define TEMPO_MAX                 2000  //Tempo máximo  ---- Em breve alterar para 1.3s
#define MIN_VOLT_ALERT            3.00  // Tensão mínima da bateria para indicar que deverá ser carregada
#define QTD_MOVIMENTOS_MAX        51  //Quantidade de movimentos
#define DELAY_FADE_MOVER          10
#define DELAY_FADE_PREPARAR       5
#define DURACAO_PREPARAR          4000

/**********************************************
 * Adição de bibliotecas, pinos e constantes  *
 * relacionadas ao LED                        *
 **********************************************/
// (c) Michael Schoeffler 2017, http://www.mschoeffler.de
#define FASTLED_ALLOW_INTERRUPTS 0
#include "FastLED.h"
#define DATA_PIN 13
#define LED_TYPE WS2812B
#define COLOR_ORDER GRB
#define NUM_LEDS 15
#define BRIGHTNESS 10
int brightness_fade = BRIGHTNESS;
char fade_direction = 0;
int delay_leds_pensar = 2*TEMPO_MAX/NUM_LEDS;
CRGB leds[NUM_LEDS];

/**********************************************
 * Definição das variáveis                    *
 **********************************************/
bool          debug = true;     // Se 'true' executa as instruções 'Serial.print***' para acompanhar processo pelo SerialMonitor. 
boolean       sentido_movimento = 0;
int           vel_mov;
unsigned long duracao_movimento;

//from https://www.electronicsblog.net/arduino-frequency-counterduty-cycle-meter/
int divider[6] = {
  0,1,8,64,256,1024};
int prescaler = 5;
unsigned int count = 0;    //variable “count”, it contains measurement of signal total period.
unsigned int middle = 0;   //variable “middle” , it contains measurement of signal high period.
int usage =0;
char x=0;

unsigned long media_velocidade = 0;
unsigned int  vetor_velocidade[QTD_MOVIMENTOS_MAX];
unsigned long idx_leit_velocidade = 0;

float corrente;               //Salva o valor medido da corrente para teste de corrente máxima
float media_corrente = 0;
float vetor_corrente[QTD_MOVIMENTOS_MAX];
int   idx_leit_corrente = 0;

// Motivo que o motou parou: 1 = fim de curso, 2 = atingiu tempo máximo, 3 = travou
unsigned int vetor_paradas[QTD_MOVIMENTOS_MAX];
unsigned int motivo_parada = 0;

// 1 = bloco pensar, 2 = bloco mover
unsigned int vetor_tipo_movimento[QTD_MOVIMENTOS_MAX];
unsigned int tipo_movimento = 0;

int idx_movimento = 0;

/**********************************************
 * Definição das funções                      *
 **********************************************/

/**********************************************
 * Função envio de mensagem inicial
 **********************************************/
void mensagem_inicial(){
  //Mensagem inicial a ser apresentada no começo do programa.
  Serial.println("Neurobots - Controle de Exoesqueleto");
  delay(1000);
  Serial.print("Aguardando comandos");
  delay(250);
  Serial.print(".");
  delay(250);
  Serial.print(".");
  delay(250);
  Serial.println(".");
  delay(250);
}

/**********************************************
 * Função envio de mensagem de parâmetros
 **********************************************/
void mensagem_parametros(){
  
  Serial.println("Parâmetros: ");
  Serial.print("Tempo de acao: ");
  Serial.println(TEMPO_MAX);
  Serial.print("Valor PWM: ");
  Serial.println(vel_mov);
}

/**********************************************
 * Função responsável pelo movimento
 **********************************************/
void pensar(){
  int pixel = 0;
  /*
   * Envio do comando mover
   */
  digitalWrite(DIRECAO, sentido_movimento);
  analogWrite(PWM_out, vel_mov);

  /*
   * Tratamento da Corrente
   */
  duracao_movimento = millis() + TEMPO_MAX;

  while ( millis() < duracao_movimento ) {
    /*
     * Tratamento da Corrente
     * Leitura e conversao da corrente do motor
     */
    corrente = analogRead(LEIT_CORRENTE);
    corrente = 1000*corrente/(1.2*1024);
    media_corrente += corrente;
    idx_leit_corrente++;

    //Teste do nível da corrente
    if (corrente > 750 ){
      motivo_parada = 3;
      
      vel_mov -= 10;
      if (vel_mov < 1) {
        vel_mov = 0;
      }
      analogWrite(PWM_out, vel_mov); 
    }

    //Efeito dos LEDs, um pixel avança a cada 'delay_leds_pensar' ms
    if ( !(millis() % delay_leds_pensar) && motivo_parada != 1 && motivo_parada != 3){
      FastLED.setBrightness(BRIGHTNESS);
      if (pixel < NUM_LEDS){
        leds[pixel-1] = CRGB::Black;
        leds[pixel] = CRGB::Blue;
        leds[pixel-2] = CRGB::Black;
        leds[pixel+1] = CRGB::Blue;
        FastLED.show();
        pixel++;
      }
    }
    
    if(motivo_parada == 1 || motivo_parada == 3){
      for (int i = 0; i < NUM_LEDS; ++i) {
        leds[i] = CRGB::Black;
      }
      FastLED.show();
    }
  }

  if ( motivo_parada == 0 ) {
    motivo_parada = 2;
  }
  analogWrite(PWM_out, 0);

}

/**********************************************
 * Função responsável pelo movimento 
 **********************************************/
void mover(){
  /*
   * Envio do comando mover
   */
  digitalWrite(DIRECAO, sentido_movimento);
  analogWrite(PWM_out, vel_mov);

  /*
   * Tratamento da Corrente
   */
  duracao_movimento = millis() + TEMPO_MAX;

  while ( millis() < duracao_movimento ) {
    /*
     * Tratamento da Corrente
     * Leitura e conversao da corrente do motor
     */
    corrente = analogRead(LEIT_CORRENTE);
    corrente = 1000*corrente/(1.2*1024);
    media_corrente += corrente;
    idx_leit_corrente++;

    //Teste do nível da corrente
    if (corrente > 750 ){
      motivo_parada = 3;
      
      vel_mov -= 10;
      if (vel_mov < 1) {
        vel_mov = 0;
      }
      analogWrite(PWM_out, vel_mov); 
    }
    
    if(motivo_parada == 1 || motivo_parada == 3){
      for (int i = 0; i < NUM_LEDS; ++i) {
        leds[i] = CRGB::Black;
      }
      FastLED.show();
    }
  }

  if ( motivo_parada == 0 ) {
    motivo_parada = 2;
  }
  analogWrite(PWM_out, 0);

  for (brightness_fade = BRIGHTNESS; brightness_fade > 0 ; brightness_fade--){
    for (int i = 0; i < NUM_LEDS; ++i) {
      FastLED.setBrightness(brightness_fade);
      leds[i] = CRGB::Blue;
      FastLED.show();
      FastLED.delay(DELAY_FADE_MOVER);
    }
  }
}

/**********************************************
 * Função parar motores
 **********************************************/
void parar_motores(){
  analogWrite(PWM_out, 0);
  delay(250);
}

/**********************************************
 * Função fim de curso
 **********************************************/
void fim_de_curso(){
  motivo_parada = 1;
  analogWrite(PWM_out, 0);
  delayMicroseconds(15000); //o maior valor que irá produzir um delay preciso é 16383. Em uma interrupção, delay() não pode ser utilizada.
}

ISR(TIMER1_OVF_vect) {

  if (prescaler<4) {
    prescaler++;
  }
}

/**********************************************
 * Rotina de Interrupção para leitura
 * da velocidade, largura do pulso.
 **********************************************/
void interrupt()
{
  if (!x) {
    count=TCNT1;
    TCNT1=0x000;  
    TCCR1B=prescaler;
    attachInterrupt(digitalPinToInterrupt(VELOCIDADE_in), interrupt, FALLING);
    media_velocidade += (16000000.0/divider[prescaler]/count);
    idx_leit_velocidade++;

  }

  else {
    middle=TCNT1;
    attachInterrupt(digitalPinToInterrupt(VELOCIDADE_in), interrupt, RISING);

  }

  x=~x; 
}

/**********************************************
 * Função para registro das médias
 * da velocidade e da corrente
 **********************************************/
void registra_media(){
//  if (idx_leit_corrente){
  if (idx_leit_corrente && idx_movimento < QTD_MOVIMENTOS_MAX){  
    
    media_velocidade /= idx_leit_velocidade;
    media_corrente /= idx_leit_corrente;
    
    vetor_velocidade[idx_movimento] = media_velocidade;
    vetor_corrente[idx_movimento] = media_corrente;
    
    vetor_paradas[idx_movimento] = motivo_parada;
    
    vetor_tipo_movimento[idx_movimento] = tipo_movimento;

    if (debug){
      Serial.println("Referente ao movimento anterior ^^^^^");
      Serial.print("Média da Velocidade: ");
      Serial.print(media_velocidade);
      Serial.println(" Hz");
  
      Serial.print("Média da corrente: ");
      Serial.print(media_corrente);
      Serial.println(" mA");

      Serial.print("*Quantidade de leituras da corrente: ");
      Serial.println(idx_leit_corrente);
      Serial.print("*Quantidade de leituras da velocidade: ");
      Serial.println(idx_leit_velocidade);
      Serial.print("*Quantidade de movimentos registrado: ");
      Serial.println(idx_movimento);
      
    }
    
    media_velocidade = 0;
    idx_leit_velocidade = 0;
    media_corrente = 0;
    idx_leit_corrente = 0;
    motivo_parada = 0;
    tipo_movimento = 0;
    idx_movimento++;
  }
}

/*********************************************
 * Preparação dos vetores para envio
 *********************************************/
void envia_medias(){
  // Monta resposta ao sw
      // Exemplo
      // {acionamentos: [{velocidade: x, corrente: y, parada: 1, tipo: 1},
      //                 {velocidade: w, corrente: z, parada: 1, tipo: 1},...]}
      int tamanho = idx_movimento;
      Serial.print("{\"acionamentos\":[");
      String texto = "";
      int i;
      for(i=0; i < tamanho; i++){
        char data[100];
        if(i != tamanho-1){
          texto = texto + "{\"velocidade\":"+("%d",vetor_velocidade[i])
                          +",\"corrente\":"+("%.2f",vetor_corrente[i])
                          +",\"parada\":"+("%d",vetor_paradas[i])
                          +",\"tipo\":"+("%d",vetor_tipo_movimento[i])+"},";
          Serial.print(texto);
          texto = "";
        }else{
          texto = texto + "{\"velocidade\":"+("%d",vetor_velocidade[i])
                          +",\"corrente\":"+("%.2f",vetor_corrente[i])
                          +",\"parada\":"+("%d",vetor_paradas[i])
                          +",\"tipo\":"+("%d",vetor_tipo_movimento[i])+"]}";
          Serial.print(texto);
        }
      }
      
      idx_movimento = 0;
      memset(vetor_velocidade, 0, tamanho*sizeof(vetor_velocidade[0]));
      memset(vetor_corrente, 0, tamanho*sizeof(vetor_corrente[0]));
      memset(vetor_paradas, 0, tamanho*sizeof(vetor_paradas[0]));
      memset(vetor_tipo_movimento, 0, tamanho*sizeof(vetor_tipo_movimento[0]));
}


/*********************************************
 * Função para apagar todos os LEDs
 *********************************************/
void showProgramCleanUp(long delayTime) {
  for (int i = 0; i < NUM_LEDS; ++i) {
    leds[i] = CRGB::Black;
  }
  FastLED.show();
  FastLED.delay(delayTime);
}

/*********************************************
 * Função de efeito: único pixel andando pela fita
 *********************************************/
void showProgramShiftSinglePixel(CRGB crgb, long delayTime) {
  for (int i = 0; i < NUM_LEDS; ++i) {
    leds[i] = crgb;
    FastLED.show();
    FastLED.delay(delayTime);
    leds[i] = CRGB::Black;
  }
}

/*********************************************
 * Função de efeito: vários pixels caminhando
 *********************************************/
void showProgramShiftMultiPixel(long delayTime) {
  for (int i = 0; i < NUM_LEDS; ++i) { 
    for (int j = i; j > 0; --j) {
      leds[j] = leds[j-1];
    }
    CRGB newPixel = CHSV(random8(), 255, 255);
    leds[0] = newPixel;
    FastLED.show();
    FastLED.delay(delayTime);
  }
}

/*********************************************
 * Função de efeito: fade in
 *********************************************/
void fade_in(CRGB crgb, long delayTime){
  for (brightness_fade = 0; brightness_fade < BRIGHTNESS ; brightness_fade++){
    for (int i = 0; i < NUM_LEDS; ++i) {
      FastLED.setBrightness(brightness_fade);
      leds[i] = crgb;
      FastLED.show();
      FastLED.delay(delayTime);
    }
  }
}

/*********************************************
 * Função de efeito: fade out
 * Ao chamar a função fade_out após o movimento,
 * o motor não funcionava quando o tipo de
 * movimento era 1 (pensar). Por isso optei 
 * por não utilizar a função, e implementar o 
 * fade out na própria função mover()
 *********************************************/
void fade_out(CRGB crgb, long delayTime){
  for (brightness_fade = BRIGHTNESS; brightness_fade > 0 ; brightness_fade--){
    for (int i = 0; i < NUM_LEDS; ++i) {
      FastLED.setBrightness(brightness_fade);
      leds[i] = crgb;
      FastLED.show();
      FastLED.delay(delayTime);
    }
  }
}

/*********************************************
 * Função de efeito: fade in out
 *********************************************/

void fade(CRGB crgb, long delayTime){
  for (int i = 0; i < NUM_LEDS; ++i) {
    FastLED.setBrightness(brightness_fade);
    leds[i] = crgb;
  }
  if (fade_direction){
    brightness_fade += 1;
    if (brightness_fade > BRIGHTNESS){
      fade_direction = ~fade_direction;
    }
  }
  else{
    brightness_fade -= 1;
    if (brightness_fade < 0) {
      brightness_fade = 0;
      fade_direction = ~fade_direction;
    }
  }
  FastLED.show();
  FastLED.delay(delayTime);
}

/*********************************************
 * Função de efeito: cor única
 *********************************************/

void color(CRGB crgb, long delayTime){
  for (int i = 0; i < NUM_LEDS; ++i) {
    FastLED.setBrightness(brightness_fade);
    leds[i] = crgb;
  }
  FastLED.show();
  FastLED.delay(delayTime);
}
/**********************************************
 * Função setup
 **********************************************/
void setup() {
  // Definiçoes para os LEDs
  FastLED.addLeds<LED_TYPE,DATA_PIN,COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip); // initializes LED strip
  FastLED.setBrightness(BRIGHTNESS);// global brightness

  // Entradas - - - - - - - - - - 
  pinMode(LEIT_CORRENTE, INPUT);
  pinMode(LEIT_PWR_IN, INPUT);
  pinMode(FIM_CURSO, INPUT);
  attachInterrupt(digitalPinToInterrupt(FIM_CURSO), fim_de_curso, RISING);
  
  pinMode(VELOCIDADE_in, INPUT);
//  digitalWrite(VELOCIDADE_in, HIGH); //resistor pull up
  TIMSK1=0x01; // enabled global and timer overflow interrupt;
  TCCR1A = 0x00; // normal operation page 148 (mode0);
  attachInterrupt(digitalPinToInterrupt(VELOCIDADE_in), interrupt, RISING);

  // Saídas  - - - - - - - - - - -
  //Motor
  pinMode(PWM_out, OUTPUT);
  pinMode(DIRECAO, OUTPUT);
  //Outros
  pinMode(LED_DADOS, OUTPUT);
  
  Serial.begin(BAUD_RATE);
  if (debug){
    mensagem_inicial();
  }
}

/**********************************************
 * Função loop
 **********************************************/
void loop() {
  String    comando;      // Leitura do comando do software
  float     volt_pwr;     //Leitura da tensão da bateria
     
  /*
   * Indicador de bateria descarregada
   * Não haverá funções 
   * de efeito com a fita de leds.
   */
  volt_pwr = ((analogRead(A4)*(5.00/1024.00))+0.12);

  /*
   * Iniciar procedimento
   */
  showProgramCleanUp(10);
  if(Serial.available()>0){
    
    //Aguardando o recebimento de algum caractere pela comunicaçao serial.
    comando = Serial.readString();

    /*
     * Comando para mover a órtese. Deve vir na forma:
     * 1º dígito = m, indica instrução de mover movimento;
     * 2º dígito = 1 ou 2, indicando o tipo de movimento (bc pensar, bc mover);
     * 3º-5º dígitos = 000 a 255, indicando o valor pwm
     */
    if(comando.length() > 1 && comando.substring(0,1)=="m"){

      //Salvando no vetor os dados do movimento anterior. 
      //OBSERVAÇÃO: Deve vir antes de todos os outros comandos desse IF, para garantir a consistência da variável tipo_movimento.
      registra_media();

      String digito_tipo_movimento = comando.substring(1,2);
      tipo_movimento = digito_tipo_movimento.toInt();
      String digitos_valor_pwm = comando.substring(2,5);
      vel_mov = digitos_valor_pwm.toInt();
      // caso o sw envie algo fora da faixa pwm admitida:
      if(vel_mov > 255){
        vel_mov = 255;
      } else if(vel_mov < 0){
        vel_mov = 0;
      }

      //Iniciar movimento
      if (debug){
        Serial.print("Movimento iniciado.\n");
//        mensagem_parametros();
      }

      sentido_movimento=!sentido_movimento;
      Serial.print("1- Sentido movimento: ");
      Serial.println(sentido_movimento);

      if ( tipo_movimento == 1 ){
        pensar();
      }else if (tipo_movimento == 2) {
        fade_in(CRGB::Blue, DELAY_FADE_MOVER);
        mover();
      }
     
      parar_motores();

      
      if (debug){
        Serial.println("Movimento finalizado.");
        Serial.println("_______________________");
      }
      
    }
    /*
     * Comando para parar
     */
    else if(comando=="s"){
      if (debug){
        Serial.println("Motores parados");
      }
//      showProgramCleanUp(1);
      parar_motores();
    }
    /*
     * Comando para envio da tensão da bateria
     */
    else if(comando=="v"){
      if(debug){
        Serial.print("Tensao da Bateria:");
        Serial.println(volt_pwr); 
      }
      String resposta = "{\"tensao\":";
      resposta = resposta + ("%.2f", volt_pwr) + "}";
      Serial.print(resposta);
    }
    /*
     * Comando para envio da velocidade e corrente
     */
    else if(comando=="r"){
      //Atribuindo os dados do último movimento para ser enviado logo após.
      registra_media();
      envia_medias();
    }
    else if(comando=="p"){
      //Efeito de fade no bloco preparar
      for (int i = 0 ; i < 3 ; i++) {
        fade_in(CRGB::Red, DELAY_FADE_PREPARAR);
//        fade_out(CRGB::Red, DELAY_FADE_PREPARAR);
      }
    }
  }

  usage=count/65536*100;
  if (prescaler>1) {
    if (usage<0.15) {
      prescaler--; 
      delay(200);
    } 
  }
}
