# kalman-fixed-agnostic

[![Crates.io](https://img.shields.io/crates/v/kalman-fixed-agnostic.svg)](https://crates.io/crates/kalman-fixed-agnostic)
[![Docs.rs](https://docs.rs/kalman-fixed-agnostic/badge.svg)](https://docs.rs/kalman-fixed-agnostic)
[![License: GPL v2](https://img.shields.io/badge/License-GPL%20v2-blue.svg)](https://www.gnu.org/licenses/old-licenses/gpl-2.0.en.html)

Filtre de Kalman adaptatif en virgule fixe pour systèmes embarqués `no_std`.

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
kalman-fixed-agnostic = "0.1.0"
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

## Licence

GPL-2.0-or-later — voir [LICENSE](LICENSE)

Copyright (C) 2026 Jorge Andre Castro