# FISHING BOAT

The goal of this project is to create a boat capable of assisting with fishing.  
I built it for fun and for my father, who goes fishing from time to time.

The mission of the boat is simple: **drop a fishing line and bait a specific area**.  
To achieve this purpose, I used the following components:

- **Nucleo STM32F446RE** : A powerful 32-bit microcontroller running up to 180 MHz, offering many connectivity interfaces and an FPU, ideal for a project involving multiple modules.

<p align="center">
  <img src="img/boat.jpeg" alt="Boat" width="170" />
</p>

- **nRF24L01** : A compact 2.4 GHz transceiver module capable of long-range wireless communication (up to 1 km with an antenna).

<p align="center">
  <img src="img/nrf24.jpg" alt="nRF24L01" width="170" />
</p>

- **Krick p285** : A 750 KV brushless motor providing good torque and efficiency.

<p align="center">
  <img src="img/s-l400.jpg" alt="Brushless motor" width="170" />
</p>

---

## Electric Schema

This section describes the overall electrical design of the fishing boat.  
The goal is to ensure **stable power distribution**, **reliable wireless communication**, and **safe motor control**.

The electrical system includes the following parts:

- **Power System:**  
  A main battery powers both the motor and the electronic boards.  
  A voltage regulator provides a clean and stable **3.3V** supply for the nRF24L01 and the STM32.  
  Filtering capacitors are added to prevent interference coming from the brushless motor.

- **Control Unit (STM32F446RE):**  
  The microcontroller serves as the central controller of the boat.  
  It generates **PWM signals** to command the brushless motor through an ESC (Electronic Speed Controller).  
  It also manages communication with the nRF24L01 via the **SPI bus**.

- **Wireless Link (nRF24L01):**  
  The module is connected through **SPI** (MOSI, MISO, SCK, CSN) and a **CE** pin to switch between TX and RX modes.  
  A capacitor (10µF to 100µF) is placed between VCC and GND to ensure stable operation and avoid signal dropouts.

- **Motor & ESC:**  
  The brushless **Krick p285 — 750 KV** motor is driven by an ESC that receives throttle commands from the STM32.  
  The ESC is powered directly from the main battery to avoid drawing too much current from the microcontroller’s regulator.

- **Safety & Filtering:**  
  Decoupling capacitors and proper grounding help to minimize electrical noise from the motor.  
  All modules share a common ground to maintain stable reference levels.

This electrical architecture ensures that the boat operates reliably, even during long-range communication or under high motor load.
