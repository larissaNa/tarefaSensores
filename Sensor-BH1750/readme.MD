# 💡 Sensor de Luminosidade (BH1750) e Controle de Servo Motor (SG90) com RP2040

Este projeto utiliza a placa **BitDogLab** com o microcontrolador **Raspberry Pi Pico (RP2040)** para **medir a intensidade da luz ambiente** com o sensor **BH1750** e reagir dinamicamente através de **LEDs RGB** e um **servo motor 9G SG90**, que se move conforme a variação da iluminação detectada.

## 🧠 Funcionalidades

- Leitura em tempo real da luminosidade (em lux) usando o sensor BH1750 via I2C.
- Exibição dos valores de luz no monitor serial.
- Iluminação de **LEDs RGB** conforme faixas de luminosidade.
- Controle de posição do **servo motor** baseado nos valores de lux (quanto mais luz, maior a posição angular do servo).

## 🔧 Componentes Utilizados

- [x] Raspberry Pi Pico (RP2040)
- [x] Sensor de luminosidade BH1750
- [x] Servo motor SG90 (9G)
- [x] BitDogLab

## 🖥️ Pinagem Utilizada

| Componente | Pino RP2040 |
|------------|-------------|
| BH1750 SDA | GP0         |
| BH1750 SCL | GP1         |
| LED Azul   | GP11        |
| LED Vermelho | GP12      |
| LED Verde  | GP13        |
| Servo SG90 | GP2         |


## 📦 Como Usar

1. **Clone o repositório:**

   ```bash
   git clone https://github.com/seu-usuario/nome-do-repositorio.git
```

2. **Compile o projeto** com o SDK do Pico (Pico SDK). Exemplo usando CMake:

   ```bash
   mkdir build
   cd build
   cmake ..
   make
   ```

3. **Grave o arquivo `.uf2`** gerado no Raspberry Pi Pico via modo bootloader.

4. **Abra o monitor serial** para visualizar os valores de lux.

## 📊 Reação dos LEDs

| Faixa de Lux (lux) | LEDs Ativos                 |
| ------------------ | --------------------------- |
| 0 – 100            | Azul                        |
| 101 – 300          | Azul + Vermelho             |
| 301 – 700          | Azul + Vermelho + Verde     |
| > 700              | Piscar o LED Verde (alerta) |

## 🔄 Controle do Servo Motor

| Faixa de Lux | Posição do Servo         |
| ------------ | ------------------------ |
| < 10         | 0° (pulso de 500us)      |
| 10 – 100     | \~45° (pulso de 1000us)  |
| 101 – 300    | \~90° (pulso de 1500us)  |
| 301 – 700    | \~135° (pulso de 1750us) |
| > 700        | \~180° (pulso de 2000us) |


## 📷 Demonstrações

### 🔌 Projeto em funcionamento

<img src="img/img1.jpg" alt="LED Verde Aceso" width="500"/>

Acima, podemos ver o LED verde aceso, indicando um nível moderado de luminosidade detectado pelo sensor BH1750.

---

### 🖥️ Leitura da luminosidade no Serial Monitor

<img src="img/img2.jpg" alt="Leitura de luminosidade no monitor serial" width="500"/>

Neste exemplo, o monitor serial mostra uma leitura de luminosidade próxima de **0 lux**, indicando um ambiente escuro. O sistema reage desligando os LEDs e posicionando o servo motor em seu ângulo mínimo.


## 🧠 Possíveis Melhorias

* Suavização da movimentação do servo (evitar saltos bruscos).
* Interface OLED para exibir lux sem depender do monitor serial.
* Adição de botões para controle manual.
