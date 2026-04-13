// Copyright (C) 2026 Jorge Andre Castro
//
// Ce programme est un logiciel libre : vous pouvez le redistribuer et/ou le modifier
// selon les termes de la Licence Publique Générale GNU telle que publiée par la
// Free Software Foundation, soit la version 2 de la licence, soit (à votre convention)
// n'importe quelle version ultérieure.

//! # kalman-fixed-agnostic
//!
//! Filtre de Kalman adaptatif en virgule fixe pour systèmes embarqués (`no_std`).
//!
//! ## Caractéristiques
//!
//! - `#![no_std]` — aucune dépendance à la bibliothèque standard
//! - Arithmétique entière pure (pas de flottants, pas de `libm`)
//! - Compatible RP2040 (Cortex-M0+), RP2350 (Cortex-M33) et architectures RISC-V
//! - Algorithme : Filtre de Kalman scalaire à modèle constant
//! - Bruit de mesure **R adaptatif** via la formule de König : `Var(X) = E[X²] − E[X]²`
//! - Temps d'exécution **constant** (déterministe) — idéal pour les noyaux temps réel
//!
//! ## Représentation interne Q16.16
//!
//! Toutes les grandeurs internes (`x`, `p`, `q`, `r`) sont stockées en format
//! virgule fixe Q16.16 sur `i64` :
//! ```text
//! valeur_réelle = valeur_i64 / 65536.0
//! ```
//! L'interface publique (entrée/sortie) reste en `i32` (entiers bruts de capteur).
//!
//! ## Algorithme
//!
//! À chaque appel à [`AgnosticKalman::update`], le filtre effectue dans l'ordre :
//!
//! 1. **Mise à jour du buffer circulaire** — la nouvelle mesure remplace la plus ancienne
//! 2. **Calcul de R dynamique** — variance de la fenêtre via la formule de König
//! 3. **Prédiction** — `P ← P + Q`
//! 4. **Gain de Kalman** — `K = P / (P + R)`
//! 5. **Correction de l'état** — `x ← x + K·(z − x)`
//! 6. **Correction de la covariance** — `P ← (1 − K)·P`
//!
//! ## Formule de König
//!
//! La variance est calculée sans passe double grâce à l'identité :
//! ```text
//! Var(X) = E[X²] − E[X]²
//! ```
//! Cette formulation est numériquement stable pour des capteurs i32 typiques
//! (ADC 12 bits, capteurs de température, accéléromètres) et s'exécute en O(N)
//! avec N = 10 (taille fixe du buffer).
//!
//! ## Exemple
//!
//! ```rust
//! use kalman_fixed_agnostic::AgnosticKalman;
//!
//! // Signal bruité autour de 1000 avec Q = 1 (filtre très lisse)
//! let mut k = AgnosticKalman::new(1000, 1);
//!
//! // Convergence progressive vers la vraie valeur
//! let mesures = [1005, 998, 1002, 1010, 995, 1003, 999, 1007, 1001, 1004];
//! let mut sortie = 0i32;
//! for &m in &mesures {
//!     sortie = k.update(m);
//! }
//!
//! // La sortie doit être proche de 1000
//! assert!((sortie - 1000).abs() < 15);
//! ```

#![no_std]

/// Taille de la fenêtre glissante pour le calcul de la variance dynamique.
 pub const WINDOW: usize = 10;
/// Filtre de Kalman adaptatif en virgule fixe Q16.16.
///
/// Le paramètre de bruit de mesure R est calculé dynamiquement à partir
/// de la variance des dernières mesures (formule de König), ce qui permet
/// au filtre de s'auto-ajuster à la qualité du signal du capteur.
pub struct AgnosticKalman {
    /// Valeur estimée en Q16.16 (valeur_réelle = x >> 16)
    x: i64,
    /// Covariance d'erreur en Q16.16
    p: i64,
    /// Bruit de processus Q en Q16.16 (confiance dans le modèle)
    q: i64,
    /// Fenêtre glissante pour le calcul de la variance dynamique
    buffer: [i32; WINDOW],
    /// Index courant dans le buffer circulaire
    buf_idx: usize,
    /// Indique si le buffer a été rempli au moins une fois
    is_warm: bool,
}

impl AgnosticKalman {
    /// Initialise un nouveau filtre avec une valeur de départ et un bruit de processus.
    ///
    /// # Arguments
    ///
    /// * `initial_value` — Valeur initiale de l'état (en unités capteur, `i32`)
    /// * `q` — Bruit de processus : confiance dans la stabilité du système.
    ///   Un `q` faible (ex. `1`) produit un filtre très lisse (réponse lente aux
    ///   changements). Un `q` élevé (ex. `100`) suit plus agressivement les mesures.
    ///
    /// # Exemple
    ///
    /// ```rust
    /// use kalman_fixed_agnostic::AgnosticKalman;
    /// let mut filtre = AgnosticKalman::new(512, 5);
    /// ```
    pub fn new(initial_value: i32, q: i32) -> Self {
        Self {
            x: (initial_value as i64) << 16,
            p: 1i64 << 16,
            q: (q as i64) << 16,
            buffer: [initial_value; WINDOW],
            buf_idx: 0,
            is_warm: false,
        }
    }

    /// Calcule la variance dynamique via la formule de König : `E[X²] − E[X]²`.
    ///
    /// Cette valeur devient le paramètre R (bruit de mesure) du filtre.
    /// Saturée à 1 (en Q16.16) pour éviter toute singularité mathématique
    /// (division par zéro dans le calcul du gain de Kalman).
    fn calculate_dynamic_r(&self) -> i64 {
        let mut sum: i64 = 0;
        let mut sum_sq: i64 = 0;

        for &val in self.buffer.iter() {
            let v = val as i64;
            sum += v;
            sum_sq += v * v;
        }

        let n = WINDOW as i64;
        let mean = sum / n;
        let mean_sq = sum_sq / n;
        // Var(X) = E[X²] − E[X]²
        let variance = mean_sq - (mean * mean);

        // Saturation basse à 1 ULP Q16.16 pour éviter K = 1 exact
        if variance > 1 {
            variance << 16
        } else {
            1i64 << 16
        }
    }

    /// Met à jour le filtre avec une nouvelle mesure brute.
    ///
    /// Exécute un cycle complet : prédiction → calcul de R → gain → correction.
    /// Le temps d'exécution est **constant** et **déterministe**.
    ///
    /// # Arguments
    ///
    /// * `measurement` — Valeur brute du capteur (`i32`)
    ///
    /// # Retour
    ///
    /// Valeur filtrée en `i32` (même échelle que l'entrée).
    ///
    /// # Exemple
    ///
    /// ```rust
    /// use kalman_fixed_agnostic::AgnosticKalman;
    /// let mut k = AgnosticKalman::new(0, 10);
    /// let v = k.update(42);
    /// assert!(v >= 0);
    /// ```
    pub fn update(&mut self, measurement: i32) -> i32 {
        // --- Mise à jour du buffer circulaire ---
        self.buffer[self.buf_idx] = measurement;
        self.buf_idx = (self.buf_idx + 1) % WINDOW;
        if self.buf_idx == 0 {
            self.is_warm = true;
        }

        // --- Calcul du paramètre R adaptatif ---
        let r = self.calculate_dynamic_r();

        // ÉTAPE 1 : PRÉDICTION — modèle à vitesse constante nulle
        // P ← P + Q
        self.p = self.p.saturating_add(self.q);

        // ÉTAPE 2 : GAIN DE KALMAN
        // K = P / (P + R), résultat en Q16.16
        let p_plus_r = self.p.saturating_add(r);
        let k = if p_plus_r != 0 {
            (self.p << 16) / p_plus_r
        } else {
            0
        };

        // ÉTAPE 3 : CORRECTION DE L'ÉTAT
        // x ← x + K · (z − x)
        let z = (measurement as i64) << 16;
        let innovation = z.saturating_sub(self.x);
        self.x = self.x.saturating_add((k * innovation) >> 16);

        // ÉTAPE 4 : CORRECTION DE LA COVARIANCE
        // P ← (1 − K) · P
        let one_q = 1i64 << 16;
        self.p = (one_q.saturating_sub(k).saturating_mul(self.p)) >> 16;
        // Saturation basse de P pour éviter la dégénérescence numérique
        if self.p < 1 {
            self.p = 1;
        }

        // Déséchélonnage vers i32
        (self.x >> 16) as i32
    }

    /// Indique si le buffer est rempli (au moins [`WINDOW`] mesures reçues).
    ///
    /// Avant le premier remplissage, la variance R est calculée sur des copies
    /// de `initial_value`, ce qui peut produire un R très bas (signal "parfait").
    /// Les applications critiques peuvent attendre `is_warm()` avant d'utiliser
    /// la sortie filtrée.
    pub fn is_warm(&self) -> bool {
        self.is_warm
    }

    /// Réinitialise l'état interne sans réallouer.
    ///
    /// Utile pour un redémarrage logiciel en place (watchdog, reprise après erreur).
    pub fn reset(&mut self, initial_value: i32) {
        self.x = (initial_value as i64) << 16;
        self.p = 1i64 << 16;
        self.buffer = [initial_value; WINDOW];
        self.buf_idx = 0;
        self.is_warm = false;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_signal_constant_converge() {
        // Un signal parfaitement constant doit être reproduit exactement après convergence
        let mut k = AgnosticKalman::new(500, 1);
        let mut out = 0i32;
        for _ in 0..50 {
            out = k.update(500);
        }
        assert_eq!(out, 500, "Signal constant : attendu 500, reçu {}", out);
    }

    #[test]
    fn test_signal_bruite_lisse() {
        // Signal bruité autour de 1000 : la sortie doit rester proche de 1000
        let mut k = AgnosticKalman::new(1000, 1);
        let bruit = [1005i32, 998, 1002, 1010, 995, 1003, 999, 1007, 1001, 1004];
        let mut out = 0i32;
        for _ in 0..5 {
            for &m in &bruit {
                out = k.update(m);
            }
        }
        assert!(
            (out - 1000).abs() < 15,
            "Signal bruité 1000 : attendu ≈1000, reçu {}",
            out
        );
    }

    #[test]
    fn test_sortie_strictement_positive_signal_positif() {
        let mut k = AgnosticKalman::new(100, 10);
        for i in 0..20 {
            let out = k.update(100 + i);
            assert!(out > 0, "Sortie doit être positive, reçu {} à i={}", out, i);
        }
    }

    #[test]
    fn test_monotone_apres_saut() {
        // Après un saut de valeur, le filtre doit converger progressivement (sans oscillation)
        let mut k = AgnosticKalman::new(0, 50);
        // Phase de chauffe à 0
        for _ in 0..10 {
            k.update(0);
        }
        // Saut à 1000
        let mut prev = k.update(1000);
        let mut monotone = true;
        for _ in 0..20 {
            let cur = k.update(1000);
            if cur < prev {
                monotone = false;
                break;
            }
            prev = cur;
        }
        assert!(monotone, "La convergence post-saut doit être monotone croissante");
    }

    #[test]
    fn test_is_warm_false_initialement() {
        let k = AgnosticKalman::new(0, 1);
        assert!(!k.is_warm(), "Le filtre ne doit pas être chaud à l'initialisation");
    }

    #[test]
    fn test_is_warm_apres_window_mesures() {
        let mut k = AgnosticKalman::new(0, 1);
        for i in 0..WINDOW {
            // Pas encore chaud après i mesures (sauf la dernière qui complète le tour)
            k.update(i as i32);
        }
        assert!(k.is_warm(), "Le filtre doit être chaud après {} mesures", WINDOW);
    }

    #[test]
    fn test_reset_reinitialise_etat() {
        let mut k = AgnosticKalman::new(1000, 5);
        for _ in 0..20 {
            k.update(2000);
        }
        k.reset(0);
        // Après reset, la première mesure doit démarrer depuis 0
        let out = k.update(0);
        assert!(
            out.abs() < 100,
            "Après reset à 0, la sortie doit être proche de 0, reçu {}",
            out
        );
        assert!(!k.is_warm(), "Après reset, is_warm doit être false");
    }

    #[test]
    fn test_signal_negatif() {
        // Le filtre doit fonctionner avec des valeurs négatives (ex. capteur bipolaire)
        let mut k = AgnosticKalman::new(-500, 5);
        let mut out = 0i32;
        for _ in 0..30 {
            out = k.update(-500);
        }
        assert!(
            (out - (-500)).abs() < 20,
            "Signal constant négatif : attendu ≈-500, reçu {}",
            out
        );
    }

    #[test]
    fn test_pas_de_saturation_sur_valeur_extreme() {
        // Vérifie qu'une mesure extrême ne provoque pas de panique (overflow)
        let mut k = AgnosticKalman::new(0, 1);
        let _ = k.update(i32::MAX / 2);
        let _ = k.update(i32::MIN / 2);
        // Pas de panique = succès
    }

    #[test]
    fn test_q_eleve_suit_signal() {
        // Avec un Q élevé, le filtre doit suivre rapidement un changement de signal
        let mut k = AgnosticKalman::new(0, 1000);
        for _ in 0..15 {
            k.update(5000);
        }
        let out = k.update(5000);
        assert!(
            (out - 5000).abs() < 200,
            "Filtre réactif (Q élevé) : attendu ≈5000, reçu {}",
            out
        );
    }
}