# STM32 Accelerometer Regression

## 📄 Project Overview  
This project demonstrates the implementation of **Linear and Polynomial Regression** on an **STM32 Nucleo-C031C6** board.  
Analog signals from an accelerometer are sampled through the on-chip **ADC (Analog-to-Digital Converter)** and processed in real time.  
The system computes regression coefficients directly on the microcontroller using raw ADC values, then sends both raw data and model predictions via **UART (Virtual COM Port)** for monitoring in a serial terminal.  

This project showcases how machine learning concepts such as regression can be applied on **resource-constrained embedded hardware**.  

---

## ✨ Features
- Sampling analog accelerometer signals using STM32 **ADC1**  
- **Linear Regression** (Y = aX + b) implementation  
- **Polynomial Regression** (Y = aX² + bX + c) implementation  
- Regression computed entirely on STM32 (no external libraries)  
- Real-time results sent over **USART2 → Virtual COM Port (VCP)**  
- Works with **Tera Term / PuTTY** for serial monitoring  

---

## 🛠 Hardware Setup
- **STM32 Nucleo-C031C6** board  
- Analog accelerometer (e.g., ADXL335, MMA7361, or equivalent)  
- Connections:  
  - ACC Pin 1 → GND  
  - ACC Pin 2 → PA0 (ADC1_IN0, X-axis)  
  - ACC Pin 3 → PA1 (ADC1_IN1, Y-axis)  
  - ACC Pin 4 → 3.3V  

---

## 📂 Project Structure
Core/

├── Inc/ # Header files

├── Src/ # Source files (main.c, adc, usart, etc.)

Drivers/ # HAL drivers

STM32C0xx/ # CMSIS and device support


---

## ▶️ How to Run
1. Clone the repo and open the project in **STM32CubeIDE**.  
2. Connect the Nucleo board via USB.  
3. Build and flash the project (`Run → Debug` or `Run`).  
4. Open **Tera Term / PuTTY** and select the COM port labeled *STLink Virtual COM Port*.  
   - Baud rate: **115200**  
   - Data: 8 bits, Parity: None, Stop: 1, Flow control: None  
5. Observe the serial output with raw ADC values, regression coefficients, and predictions.  

---

## 📊 Example Output
Linear regression: Y = 0.982 * X + 10.523

X=2259, Y_real=2130, Y_pred=2259.66, Error=-129.66

X=2260, Y_real=2130, Y_pred=2260.65, Error=-130.65


---

## 📌 Future Improvements
- Implement higher-order polynomial regression (3rd or 4th degree)  
- Add support for real-time error metrics (MSE, RMSE)  
- Visualize results via UART → Python plotting script  

---

## 🔄 System Data Flow

<img width="986" height="142" alt="image" src="https://github.com/user-attachments/assets/b89697ca-11a6-4676-a4b0-9122f00c5dd9" />


---

## Technical Documentation

You can find technical documentation [here](Docs/TehničkaDokumentacija.pdf).

---

## 👥 Contributors
This project was developed as part of the **Microprocessor Systems** course (*Mikroprocesorski sistemi*).

- [Marija Jolović](https://github.com/marijajolovic) — 46/2021  
- [Stefan Stanišić](https://github.com/stanisicstefan) — 81/2021  
- [Anđelina Maksimović](https://github.com/AndjelinaMaksimovic) — 56/2021  

