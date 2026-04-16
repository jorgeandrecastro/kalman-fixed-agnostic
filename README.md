# kalman-fixed-agnostic

[![Crates.io](https://img.shields.io/crates/v/kalman-fixed-agnostic.svg)](https://crates.io/crates/kalman-fixed-agnostic)
[![Docs.rs](https://docs.rs/kalman-fixed-agnostic/badge.svg)](https://docs.rs/kalman-fixed-agnostic)
[![License: GPL v2](https://img.shields.io/badge/License-GPL%20v2-blue.svg)](https://www.gnu.org/licenses/old-licenses/gpl-2.0.en.html)

Filtre de Kalman adaptatif en virgule fixe pour systèmes embarqués `no_std`.

# Update Version 0.2.0
#![forbid(unsafe_code)] pour empecher le code  unsafe.

## Caractéristiques

- `#![no_std]` — aucune dépendance à la bibliothèque standard
- Arithmétique entière pure (pas de flottants, pas de `libm`)
- Compatible RP2040 (Cortex-M0+), RP2350 (Cortex-M33) et architectures RISC-V
- Bruit de mesure **R adaptatif** calculé dynamiquement via la **formule de König**
- Temps d'exécution **constant** et **déterministe** — idéal pour les noyaux RTOS
- Interface simple : `new(valeur_initiale, q)` → `update(mesure)` → `i32`

## Utilisation

```toml
[dependencies]
kalman-fixed-agnostic = "0.1.1"
```

```rust
use kalman_fixed_agnostic::AgnosticKalman;

// Initialisation avec valeur de départ et bruit de processus Q
let mut filtre = AgnosticKalman::new(1000, 5);

// Boucle de mesure
loop {
    let brut: i32 = lire_capteur();
    let filtre_val = filtre.update(brut);
    utiliser(filtre_val);
}
```

## Algorithme

### Cycle de Kalman

À chaque appel à `update()` :

```
1. Buffer circulaire ← nouvelle mesure
2. R = Var(buffer) via formule de König : E[X²] − E[X]²
3. P ← P + Q                          (prédiction)
4. K = P / (P + R)                    (gain de Kalman)
5. x ← x + K · (z − x)               (correction état)
6. P ← (1 − K) · P                   (correction covariance)
```

### Formule de König

La variance est calculée en un seul passage sur le buffer de taille 10 :

```
Var(X) = E[X²] − E[X]²
```

Cela évite la double boucle et reste exact pour des valeurs `i32` typiques
(ADC 12 bits, capteurs de température, accéléromètres).

### Format interne Q16.16

Les grandeurs internes sont stockées sur `i64` en format Q16.16 :

```
valeur_réelle = valeur_i64 / 65536
```

L'interface publique reste en `i32` (unités capteur brutes).

## Paramètre Q

| Valeur Q | Comportement |
|----------|--------------|
| `1` | Très lisse, réponse lente aux changements |
| `10` | Équilibre lissage / réactivité |
| `100`+ | Très réactif, suit les variations rapidement |

## Méthodes

| Méthode | Description |
|---------|-------------|
| `new(initial, q)` | Crée un filtre initialisé |
| `update(mesure)` | Met à jour et retourne la valeur filtrée |
| `is_warm()` | `true` après 10 mesures (buffer rempli une fois) |
| `reset(valeur)` | Réinitialise sans réallocation (reprise watchdog) |


# 🚀 Exemple Complet : Filtrage de Kalman & Multitâche
Cet exemple démontre comment utiliser le driver avec le filtre de Kalman adaptatif pour obtenir une stabilité parfaite, même sur le RP2350 (Pico 2).
```rust 
#![no_std]
#![no_main]

use cortex_m_rt as _;
use embassy_executor::Spawner;
use embassy_rp::gpio::{Level, Output, Flex, Pull};
use embassy_rp::i2c::{Config as I2cConfig, I2c, Async};
use embassy_time::{Delay, Duration, Timer};
use hd44780_i2c_nostd::LcdI2c;
use {panic_halt as _, embassy_rp as _};
use core::fmt::Write;
use heapless::String;

// Drivers & Kalman 🦅
use embassy_am2302::{am2302_read, EnvData};
use embassy_am2302::signals::ENV_SIGNAL;
use kalman_fixed_agnostic::AgnosticKalman;

use rp2350_linker as _;
use embassy_rp::bind_interrupts;
use embassy_rp::peripherals::I2C0;

bind_interrupts!(struct Irqs {
    I2C0_IRQ => embassy_rp::i2c::InterruptHandler<I2C0>;
});

// --- TASK 1 : LECTURE DU CAPTEUR + FILTRAGE ADAPTATIF ---
#[embassy_executor::task]
async fn sensor_task(mut pin: Flex<'static>) {
    // Initialisation des filtres (x10 pour garder une décimale en i32)
    // Q = 2 : Filtre très stable pour la température
    // Q = 5 : Un peu plus réactif pour l'humidité
    let mut temp_kalman = AgnosticKalman::new(200, 2); 
    let mut hum_kalman = AgnosticKalman::new(500, 5);

    loop {
        match am2302_read(&mut pin, 120).await { 
            Ok(data) => {
                // Conversion f32 -> i32 (x10)
                let t_raw = (data.temp * 10.0) as i32;
                let h_raw = (data.hum * 10.0) as i32;

                // Application du filtre de Kalman (König-Huygens inside)
                let t_filtered = temp_kalman.update(t_raw);
                let h_filtered = hum_kalman.update(h_raw);

                // Envoi des données lissées au reste du système
                ENV_SIGNAL.signal(EnvData {
                    temp: t_filtered as f32 / 10.0,
                    hum: h_filtered as f32 / 10.0,
                });
            }
            Err(_) => {
                ENV_SIGNAL.signal(EnvData { temp: 999.0, hum: 0.0 });
            }
        }
        Timer::after(Duration::from_millis(2500)).await;
    }
}

// --- TASK 2 : AFFICHAGE LCD ---
#[embassy_executor::task]
async fn display_task(mut lcd: LcdI2c<I2c<'static, I2C0, Async>>) {
    let mut delay = Delay;
    Timer::after(Duration::from_millis(500)).await;
    
    if lcd.init(&mut delay).await.is_ok() {
        let _ = lcd.set_backlight(true);
        let _ = lcd.clear(&mut delay).await;
        let _ = lcd.write_str("  JC-OS KERNEL", &mut delay).await;
        let _ = lcd.set_cursor(1, 0, &mut delay).await;
        let _ = lcd.write_str("  KALMAN ON 🦅", &mut delay).await;
    }

    loop {
        let data = ENV_SIGNAL.wait().await;
        let _ = lcd.clear(&mut delay).await;
        let mut s: String<16> = String::new();

        if data.temp > 500.0 {
            let _ = lcd.set_cursor(0, 0, &mut delay).await;
            let _ = lcd.write_str("SENSOR ERROR", &mut delay).await;
        } else {
            // Affichage Température
            let _ = write!(s, "TEMP: {:.1} C", data.temp);
            let _ = lcd.set_cursor(0, 0, &mut delay).await;
            let _ = lcd.write_str(s.as_str(), &mut delay).await;

            // Affichage Humidité
            s.clear();
            let _ = write!(s, "HUM : {:.1} %", data.hum);
            let _ = lcd.set_cursor(1, 0, &mut delay).await;
            let _ = lcd.write_str(s.as_str(), &mut delay).await;
        }
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(embassy_rp::config::Config::default());

    // Setup I2C & LCD
    let mut i2c_config = I2cConfig::default();
    i2c_config.frequency = 100_000;
    let i2c = I2c::new_async(p.I2C0, p.PIN_5, p.PIN_4, Irqs, i2c_config);
    let lcd = LcdI2c::new(i2c, 0x3F); 

    // Setup Capteur
    let mut sensor_pin = Flex::new(p.PIN_22);
    sensor_pin.set_pull(Pull::Up);
    sensor_pin.set_as_output();
    sensor_pin.set_high();
    Timer::after_millis(100).await; 

    // Launch tasks
    spawner.spawn(sensor_task(sensor_pin)).unwrap();
    spawner.spawn(display_task(lcd)).unwrap();

    // Heartbeat LED
    let mut led = Output::new(p.PIN_25, Level::Low);
    loop {
        led.toggle();
        Timer::after_millis(500).await;
    }
}

```

Cet exmple utilise mes autres crates vous avez donc une vue d'ensemble La oled , le capteur et le filtre de Kalman :)





## Licence

GPL-2.0-or-later — voir [LICENSE](LICENSE)

Copyright (C) 2026 Jorge Andre Castro